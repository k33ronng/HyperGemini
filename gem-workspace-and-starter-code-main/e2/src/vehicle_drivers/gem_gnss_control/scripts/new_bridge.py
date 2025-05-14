#!/usr/bin/env python3

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDrive
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, VehicleSpeedRpt
from std_msgs.msg import Bool

class GEMBridgePID:
    def __init__(self):
        rospy.init_node("gem_pacmod_bridge_pid")

        # PID gains
        self.Kp = rospy.get_param("~Kp", 0.5)
        self.Ki = rospy.get_param("~Ki", 0.0)
        self.Kd = rospy.get_param("~Kd", 0.1)

        # State
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = rospy.Time.now()

        self.target_speed = 0.0
        self.current_speed = 0.0
        self.current_gear = None

        # Publishers
        self.enable_pub = rospy.Publisher("/pacmod/as_rx/enable", Bool, queue_size=1)
        self.gear_pub   = rospy.Publisher("/pacmod/as_rx/shift_cmd", PacmodCmd, queue_size=1)
        self.accel_pub  = rospy.Publisher("/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size=1)
        self.brake_pub  = rospy.Publisher("/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size=1)
        self.steer_pub  = rospy.Publisher("/pacmod/as_rx/steer_cmd", PositionWithSpeed, queue_size=1)

        # Subscribers
        rospy.Subscriber("/gem/stanley_gnss_cmd", AckermannDrive, self.cmd_callback)
        rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)

        # Start
        rospy.sleep(1.0)
        self.enable_pub.publish(Bool(data=True))
        rospy.loginfo("✅ GEM PACMod PID bridge initialized.")
        rospy.spin()

    def front2steer(self, front_angle_rad):
        max_steer_rad = np.radians(30)
        front_angle_rad = np.clip(front_angle_rad, -max_steer_rad, max_steer_rad)
        return np.sign(front_angle_rad) * (0.6 * abs(front_angle_rad) + 0.3 * abs(front_angle_rad) ** 2)

    def speed_callback(self, msg):
        self.current_speed = msg.vehicle_speed  # always positive

    def cmd_callback(self, msg):
        self.target_speed = msg.speed
        steer_front_rad = msg.steering_angle

        # === Gear control ===
        if self.target_speed > 0:
            self.set_gear(3)  # DRIVE
        elif self.target_speed < 0:
            self.set_gear(1)  # REVERSE

        # === Steering control ===
        wheel_angle = self.front2steer(steer_front_rad)
        steer_msg = PositionWithSpeed()
        steer_msg.angular_position = wheel_angle
        steer_msg.angular_velocity_limit = 1.0
        self.steer_pub.publish(steer_msg)

        # === PID throttle/brake ===
        self.publish_throttle_brake()

    def set_gear(self, gear_code):
        if self.current_gear != gear_code:
            self.gear_pub.publish(PacmodCmd(ui16_cmd=gear_code))
            self.current_gear = gear_code
            gname = "DRIVE" if gear_code == 3 else "REVERSE"
            rospy.loginfo(f"[BridgePID] Gear set to {gname}")

    def publish_throttle_brake(self):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt <= 0:
            return

        # PID computation
        error = abs(self.target_speed) - self.current_speed
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        control = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        control = np.clip(control, 0.0, 0.5)

        # Send commands
        if abs(self.target_speed) < 0.05:
            self.brake_pub.publish(PacmodCmd(f64_cmd=0.5))
            self.accel_pub.publish(PacmodCmd(f64_cmd=0.0))
        else:
            self.brake_pub.publish(PacmodCmd(f64_cmd=0.0))
            self.accel_pub.publish(PacmodCmd(f64_cmd=control))

        rospy.loginfo_throttle(1.0, f"[BridgePID] Target={self.target_speed:.2f} | Current={self.current_speed:.2f} | Accel={control:.2f}")

if __name__ == "__main__":
    try:
        GEMBridgePID()
    except rospy.ROSInterruptException:
        pass

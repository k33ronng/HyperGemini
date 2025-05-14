#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDrive
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, VehicleSpeedRpt
from std_msgs.msg import Bool
import numpy as np
import time

class GEMBridgePID:
    def __init__(self):
        rospy.init_node("gem_pacmod_bridge_pid")

        # Control parameters
        self.Kp = rospy.get_param("~Kp", 1)
        self.Ki = rospy.get_param("~Ki", 0)
        self.Kd = rospy.get_param("~Kd", 0.1)

        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = rospy.Time.now()

        # State
        self.target_speed = 0.0
        self.steering_angle = 0.0
        self.current_speed = 0.0
        self.current_gear = None

        # ROS interfaces
        self.enable_pub = rospy.Publisher("/pacmod/as_rx/enable", Bool, queue_size=1)
        self.gear_pub   = rospy.Publisher("/pacmod/as_rx/shift_cmd", PacmodCmd, queue_size=1)
        self.accel_pub  = rospy.Publisher("/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size=1)
        self.brake_pub  = rospy.Publisher("/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size=1)
        self.steer_pub  = rospy.Publisher("/pacmod/as_rx/steer_cmd", PositionWithSpeed, queue_size=1)

        rospy.Subscriber("/gem/stanley_gnss_cmd", AckermannDrive, self.cmd_callback)
        rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)

        rospy.sleep(1.0)
        self.enable_pub.publish(Bool(data=True))
        rospy.loginfo("✅ PACMod bridge with PID speed controller started.")
        rospy.spin()

    def speed_callback(self, msg):
        self.current_speed = msg.vehicle_speed  # Always positive

    def cmd_callback(self, msg):
        self.target_speed = msg.speed
        self.steering_angle = msg.steering_angle  # Assumes radians, front wheel

        # Gear selection
        if self.target_speed >= 0:
            self.set_gear(3)  # DRIVE
        else:
            self.set_gear(1)  # REVERSE

        self.publish_steering(self.steering_angle)
        self.publish_throttle_brake()

    def set_gear(self, gear_code):
        if self.current_gear != gear_code:
            self.gear_pub.publish(PacmodCmd(ui16_cmd=gear_code))
            self.current_gear = gear_code
            gname = "DRIVE" if gear_code == 3 else "REVERSE"
            rospy.loginfo(f"[Bridge] Gear set to {gname}")

    def publish_steering(self, front_angle_rad):
        max_steer_rad = (30)
        front_angle_rad = np.clip(front_angle_rad, -max_steer_rad, max_steer_rad)
        wheel_angle = np.sign(front_angle_rad) * (0.6 * abs(front_angle_rad) + 0.3 * abs(front_angle_rad)**2)

        msg = PositionWithSpeed()
        msg.angular_position = wheel_angle
        msg.angular_velocity_limit = 2.5
        self.steer_pub.publish(msg)

    def publish_throttle_brake(self):
        # PID control
        error = abs(self.target_speed) - (self.current_speed)
        dt = (rospy.Time.now() - self.last_time).to_sec()
        self.last_time = rospy.Time.now()

        if dt <= 0:
            return

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        control = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        control = np.clip(control, 0, 0.5)

        if abs(self.target_speed) < 0.05:
            # Apply brake if nearly stopped
            self.brake_pub.publish(PacmodCmd(f64_cmd=0.5))
            self.accel_pub.publish(PacmodCmd(f64_cmd=0.0))
        else:
            # Compute sign-aware acceleration
            self.brake_pub.publish(PacmodCmd(f64_cmd=0.0))
            self.accel_pub.publish(PacmodCmd(f64_cmd=(control)))


        rospy.loginfo_throttle(1.0, f"[BridgePID] Target: {self.target_speed:.2f}, Current: {self.current_speed:.2f}, Accel: {control:.2f}")

if __name__ == "__main__":
    try:
        GEMBridgePID()
    except rospy.ROSInterruptException:
        pass

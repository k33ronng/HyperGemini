#!/usr/bin/env python3

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDrive
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, VehicleSpeedRpt
from std_msgs.msg import Bool

class GEMBridgePID:
    def __init__(self):
        rospy.init_node("gem_pacmod_topic_bridge_pid")

        # PID gains
        self.Kp = rospy.get_param("~Kp", 1)
        self.Ki = rospy.get_param("~Ki", 0.0)
        self.Kd = rospy.get_param("~Kd", 0.1)

        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = rospy.Time.now()

        self.target_speed = 0.0
        self.current_speed = 0.0
        self.current_gear = None

        # Publishers
        self.enable_pub = rospy.Publisher("/pacmod/as_rx/enable", Bool, queue_size=1)
        self.gear_pub   = rospy.Publisher("/pacmod/as_rx/shift_cmd", PacmodCmd, queue_size=1)
        self.brake_pub  = rospy.Publisher("/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size=1)
        self.accel_pub  = rospy.Publisher("/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size=1)
        self.steer_pub  = rospy.Publisher("/pacmod/as_rx/steer_cmd", PositionWithSpeed, queue_size=1)

        # Subscribers
        rospy.Subscriber("/gem/stanley_gnss_cmd", AckermannDrive, self.cmd_callback)
        rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)

        # Enable control
        rospy.sleep(1.0)
        self.enable_pub.publish(Bool(data=True))
        rospy.loginfo("✅ GEM PACMod PID bridge initialized.")

    def front2steer(self, front_angle_rad):
        return np.sign(front_angle_rad) * (0.6 * abs(front_angle_rad) + 0.3 * abs(front_angle_rad) ** 2)

    def speed_callback(self, msg):
        self.current_speed = msg.vehicle_speed  # always positive

    def cmd_callback(self, msg):
        speed = msg.speed
        steer = msg.steering_angle
        self.target_speed = speed

        # Gear selection
        if self.target_speed >= 0:
            new_gear = 3  # DRIVE
        else:
            new_gear = 1  # REVERSE

        if new_gear != self.current_gear:
            self.set_gear(new_gear)
            rospy.sleep(0.5)  # Allow time for gear shift

        # === Throttle/brake ===
        accel_cmd = self.compute_pid_accel(speed, self.current_speed)

        if abs(speed) < 0.05:
            self.brake_pub.publish(PacmodCmd(f64_cmd=0.5))
            self.accel_pub.publish(PacmodCmd(f64_cmd=0.0))
        else:
            self.brake_pub.publish(PacmodCmd(f64_cmd=0.0))
            self.accel_pub.publish(PacmodCmd(f64_cmd=accel_cmd))

        # === Steering ===
        steer_msg = PositionWithSpeed()
        steer_msg.angular_position = self.front2steer(steer)
        steer_msg.angular_velocity_limit = 3
        self.steer_pub.publish(steer_msg)

        rospy.loginfo_throttle(1.0, f"[BridgePID] Gear={self.current_gear} | Target={speed:.2f} | Actual={self.current_speed:.2f} | Accel={accel_cmd:.2f}")

    def set_gear(self, gear_code):
        if self.current_gear != gear_code:
            self.gear_pub.publish(PacmodCmd(ui16_cmd=gear_code))
            self.current_gear = gear_code
            gname = "DRIVE" if gear_code == 3 else "REVERSE"
            rospy.loginfo(f"🕹️ Gear set to {gname}")

    def compute_pid_accel(self, target, actual):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt <= 0:
            return 0.0

        # Use current gear to determine error
        if self.current_gear == 3:  # DRIVE
            error = target - actual
        elif self.current_gear == 1:  # REVERSE
            target_mag = abs(target)
            error = target_mag - actual  # Compare magnitudes in reverse
        else:
            error = 0.0  # Neutral/park; no throttle

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return np.clip(output, 0.0, 0.5)
    
            # === DRIVE: PID Control ===
        if self.current_gear == 3:
            error = target - actual
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            self.prev_error = error

            output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            return np.clip(output, 0.0, 0.5)

        # === REVERSE: Linear Mapping ===
        elif self.current_gear == 1:
            # Use simple linear map: speed / max_speed
            max_reverse_speed = 1.5  # m/s (adjust as needed)
            accel_cmd = np.clip(abs(target) / max_reverse_speed, 0.0, 0.4)
            # zero accel_cmd if current speed is greater than target speed
            if actual > target:
                accel_cmd = 0.0
        
            return accel_cmd
        return 0.0

if __name__ == "__main__":
    try:
        GEMBridgePID()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

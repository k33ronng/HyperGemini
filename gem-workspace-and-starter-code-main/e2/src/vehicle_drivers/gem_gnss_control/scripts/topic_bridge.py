#!/usr/bin/env python3

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDrive
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed
from std_msgs.msg import Bool

class GEMBridgeNode:
    def __init__(self):
        rospy.init_node("gem_pacmod_bridge", anonymous=True)

        # Publishers
        self.enable_pub = rospy.Publisher("/pacmod/as_rx/enable", Bool, queue_size=1)
        self.gear_pub   = rospy.Publisher("/pacmod/as_rx/shift_cmd", PacmodCmd, queue_size=1)
        self.accel_pub  = rospy.Publisher("/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size=1)
        self.brake_pub  = rospy.Publisher("/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size=1)
        self.steer_pub  = rospy.Publisher("/pacmod/as_rx/steer_cmd", PositionWithSpeed, queue_size=1)

        # State tracking
        self.current_gear = None  # 3 = DRIVE, 4 = REVERSE

        rospy.Subscriber("/gem/stanley_gnss_cmd", AckermannDrive, self.cmd_callback)

        rospy.sleep(1.0)
        self.enable_pub.publish(Bool(data=True))
        rospy.sleep(0.2)
        self.set_gear(3)  # Default to DRIVE

        rospy.loginfo("✅ GEM PACMod bridge (PID-compatible) initialized.")
        rospy.spin()

    def front2steer(self, front_angle_rad):
        """Map front wheel angle to steering wheel angle."""
        max_steer_rad = (30)  # approx limit
        front_angle_rad = np.clip(front_angle_rad, -max_steer_rad, max_steer_rad)
        # Inverse of the polynomial from PID script
        steer_angle = np.sign(front_angle_rad) * (0.6 * abs(front_angle_rad) + 0.3 * abs(front_angle_rad) ** 2)
        return steer_angle

    def set_gear(self, gear_code):
        if self.current_gear != gear_code:
            self.gear_pub.publish(PacmodCmd(ui16_cmd=gear_code))
            self.current_gear = gear_code
            gname = "DRIVE" if gear_code == 3 else "REVERSE"
            rospy.loginfo(f"[Bridge] Gear set to {gname}")

    def cmd_callback(self, msg):
        speed = msg.speed
        steer_front_rad = msg.steering_angle

        # Handle gear and acceleration
        if abs(speed) < 0.05:
            brake_cmd = 0.5
            accel_cmd = 0.0
        else:
            if speed > 0:
                self.set_gear(3)
                brake_cmd = 0.0
                accel_cmd = np.clip(speed / 2, 0.0, 0.4)
            else:
                self.set_gear(1)
                brake_cmd = 0.0
                accel_cmd = np.clip(abs(speed) / 2, 0.0, 0.4)

        # Apply same steering transform as PID script
        steer_wheel_angle = self.front2steer(steer_front_rad)

        steer_msg = PositionWithSpeed()
        steer_msg.angular_position = steer_wheel_angle
        steer_msg.angular_velocity_limit = 1.0

        # Publish commands
        self.accel_pub.publish(PacmodCmd(f64_cmd=accel_cmd))
        self.brake_pub.publish(PacmodCmd(f64_cmd=brake_cmd))
        self.steer_pub.publish(steer_msg)

        rospy.loginfo_throttle(1.0, f"[Bridge] Speed={speed:.2f} m/s → Accel={accel_cmd:.2f}, Brake={brake_cmd:.2f}, Steer(front)={np.degrees(steer_front_rad):.1f}°, Steer(wheel)={np.degrees(steer_wheel_angle):.1f}°")

if __name__ == "__main__":
    try:
        GEMBridgeNode()
    except rospy.ROSInterruptException:
        pass

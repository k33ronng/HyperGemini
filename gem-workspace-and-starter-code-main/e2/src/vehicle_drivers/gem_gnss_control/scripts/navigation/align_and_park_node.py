#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from tf.transformations import euler_from_quaternion
import numpy as np

# Import your two parking controllers
from parallel_func import SimpleParallelPark
from diagonal_func import SimpleDiagonalPark

class AlignAndPark:
    def __init__(self):
        rospy.init_node("align_and_park_node")

        # === Configurable Parameters ===
        self.entry_pose = rospy.get_param("~entry_pose", [5.0, -2.0, 0.0])  # x, y, yaw
        self.mode = rospy.get_param("~parking_mode", "parallel")  # "parallel" or "diagonal"
        self.tolerance_pos = 0.2
        self.tolerance_yaw = 0.1  # radians

        self.Kp_lin = 1.0
        self.Kp_ang = 1.5
        self.max_speed = 0.8
        self.max_steer = 30.0

        self.current_pose = None
        self.state = "align"

        self.cmd_pub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size=10)
        rospy.Subscriber("/odometry", Odometry, self.odom_callback)

        rospy.loginfo(f"[AlignAndPark] Initialized in {self.mode.upper()} mode.")
        self.rate = rospy.Rate(10)
        self.main_loop()

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
        self.current_pose = (pos.x, pos.y, yaw)

    def normalize(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def main_loop(self):
        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.loginfo_throttle(2.0, "⏳ Waiting for odometry...")
                self.rate.sleep()
                continue

            x, y, yaw = self.current_pose
            gx, gy, gyaw = self.entry_pose

            dx = gx - x
            dy = gy - y
            dist = math.hypot(dx, dy)
            desired_heading = math.atan2(dy, dx)
            heading_error = self.normalize(desired_heading - yaw)
            yaw_error = self.normalize(gyaw - yaw)

            cmd = AckermannDrive()

            if self.state == "align":
                if dist > self.tolerance_pos or abs(yaw_error) > self.tolerance_yaw:
                    cmd.speed = min(self.max_speed, self.Kp_lin * dist)
                    cmd.steering_angle = np.clip(math.degrees(self.Kp_ang * heading_error), -self.max_steer, self.max_steer)
                    rospy.loginfo_throttle(1.0, f"[Aligning] dist={dist:.2f}, yaw_err={math.degrees(yaw_error):.1f}°")
                else:
                    rospy.loginfo("✅ Aligned to entry pose. Executing parking maneuver...")
                    self.state = "parking"
                    self.trigger_parking()
                    continue

            elif self.state == "done":
                cmd = AckermannDrive()

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

    def trigger_parking(self):
        if self.mode == "parallel":
            rospy.loginfo("🚗 Starting Parallel Parking")
            SimpleParallelPark().run()
        elif self.mode == "diagonal":
            rospy.loginfo("🚗 Starting Diagonal Parking")
            SimpleDiagonalPark().run()
        else:
            rospy.logwarn("⚠️ Unknown parking mode specified. Skipping.")
        self.state = "done"

if __name__ == "__main__":
    try:
        AlignAndPark()
    except rospy.ROSInterruptException:
        pass

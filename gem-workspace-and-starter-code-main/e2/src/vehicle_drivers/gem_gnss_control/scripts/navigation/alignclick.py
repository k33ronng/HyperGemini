#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from tf.transformations import euler_from_quaternion
import numpy as np

from hardcode_para import SimpleParallelPark
from diagonal import SimpleDiagonalPark  # Update to match your filename

class AlignAndPark:
    def __init__(self):
        rospy.init_node("align_and_park_node")

        self.mode = rospy.get_param("~parking_mode", "parallel")  # or "diagonal"
        self.tolerance_pos = 0.2
        self.tolerance_yaw = 0.2  # radians

        self.Kp_lin = 1.0
        self.Kp_ang = 0.3
        self.max_speed = 0.8
        self.max_steer = 20

        self.current_pose = None
        self.goal_pose = None
        self.state = "waiting"

        self.cmd_pub = rospy.Publisher("/gem/stanley_gnss_cmd", AckermannDrive, queue_size=10)
        rospy.Subscriber("/Odometry", Odometry, self.odom_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        rospy.loginfo("🟢 Align and Park node ready. Click a goal in RViz.")
        self.rate = rospy.Rate(10)
        self.main_loop()

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
        self.current_pose = (pos.x, pos.y, yaw)

    def goal_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
        self.goal_pose = (pos.x, pos.y, yaw) # change the goal to the relative position
        self.state = "align"
        rospy.loginfo(f"📍 New goal received: x={pos.x:.2f}, y={pos.y:.2f}, yaw={math.degrees(yaw):.1f}°")

    def normalize(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def main_loop(self):
        while not rospy.is_shutdown():
            if self.current_pose is None or self.goal_pose is None:
                self.rate.sleep()
                continue

            x, y, yaw = self.current_pose
            gx, gy, gyaw = self.goal_pose

            dx = gx - x
            dy = gy - y
            dist = math.hypot(dx, dy)
            desired_heading = math.atan2(dy, dx)
            heading_error = self.normalize(desired_heading - yaw)
            yaw_error = self.normalize(gyaw - yaw)

            cmd = AckermannDrive()

            if self.state == "align":
                if dist > self.tolerance_pos or abs(yaw_error) > self.tolerance_yaw:
                    cmd.speed = min(self.Kp_lin * dist, self.max_speed)
                    cmd.steering_angle = np.clip(math.degrees(self.Kp_ang * heading_error), -self.max_steer, self.max_steer)
                    rospy.loginfo_throttle(1.0, f"[Aligning] dist={dist:.2f}, yaw_err={math.degrees(yaw_error):.1f}°")
                else:
                    rospy.loginfo("✅ Aligned. Starting parking maneuver...")
                    self.state = "parking"
                    # self.trigger_parking()
                    continue

            elif self.state == "done":
                cmd = AckermannDrive()

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

    def trigger_parking(self):
        if self.mode == "parallel":
            rospy.loginfo("🚗 Executing SimpleParallelPark")
            SimpleParallelPark()
        elif self.mode == "diagonal":
            rospy.loginfo("🚗 Executing SimpleDiagonalPark")
            SimpleDiagonalPark()
        else:
            rospy.logwarn("❌ Unknown parking mode. Skipping.")
        self.state = "done"

if __name__ == "__main__":
    try:
        AlignAndPark()
    except rospy.ROSInterruptException:
        pass

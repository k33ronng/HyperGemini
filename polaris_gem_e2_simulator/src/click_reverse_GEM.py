#!/usr/bin/env python3

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from tf.transformations import euler_from_quaternion

from hardcode_para import SimpleParallelPark
from diagonal import SimpleDiagonalPark

class AlignAndPark:
    def __init__(self):
        rospy.init_node("align_and_park_node")

        self.mode = rospy.get_param("~parking_mode", "parallel")  # or "diagonal"
        self.dist_thresh = 0.2
        self.yaw_thresh = 0.2  # radians

        self.Kp_speed = 1.0
        self.Kp_steer = 0.3
        self.align_radius = 3.0

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
        self.goal_pose = (pos.x, pos.y, yaw)
        self.state = "approach"
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
            distance = math.hypot(dx, dy)
            heading_to_goal = math.atan2(dy, dx)
            heading_error = self.normalize(heading_to_goal - yaw)
            final_yaw_error = self.normalize(gyaw - yaw)

            cmd = AckermannDrive()

            if self.state == "approach":
                if abs(heading_error) > math.pi / 2 and distance > 2.0:
                    rospy.loginfo(" Goal is behind. Switching to reverse.")
                    self.state = "reverse_approach"
                    continue

                if distance > self.dist_thresh:
                    if distance < self.align_radius:
                        cmd.speed = min(self.Kp_speed * distance, 0.5)
                        cmd.steering_angle = np.clip(self.Kp_steer * final_yaw_error, -30.0, 30.0)
                        rospy.loginfo_throttle(1.0, f"[BlendAlign] dist={distance:.2f}, yaw_err={math.degrees(final_yaw_error):.1f}°")
                    else:
                        cmd.speed = min(self.Kp_speed * distance, 1.0)
                        cmd.steering_angle = np.clip(self.Kp_steer * heading_error, -30.0, 30.0)
                        rospy.loginfo_throttle(1.0, f"[Approach] dist={distance:.2f}, heading_err={math.degrees(heading_error):.1f}°")
                else:
                    rospy.loginfo("Reached goal position. Aligning heading...")
                    self.state = "align"

            elif self.state == "reverse_approach":
                if distance > self.dist_thresh:
                    adjusted_heading = self.normalize(heading_error + math.pi)
                    cmd.speed = -min(self.Kp_speed * distance, 0.5)
                    cmd.steering_angle = np.clip(-self.Kp_steer * adjusted_heading, -30.0, 30.0)
                    rospy.loginfo_throttle(1.0, f"[Reverse] dist={distance:.2f}, reverse_yaw_err={math.degrees(adjusted_heading):.1f}°")
                else:
                    rospy.loginfo(" Reached goal in reverse. Aligning heading...")
                    self.state = "reverse_align"

            elif self.state == "align":
                if abs(final_yaw_error) > self.yaw_thresh:
                    cmd.speed = 0.2
                    cmd.steering_angle = np.clip(self.Kp_steer * final_yaw_error, -30.0, 30.0)
                    rospy.loginfo_throttle(1.0, f"[Align] yaw_err={math.degrees(final_yaw_error):.1f}°")
                else:
                    rospy.loginfo("Heading aligned. Fine tuning position...")
                    self.state = "final_approach"
            elif self.state == "final_approach":
                gx, gy, _ = self.goal_pose
                dx = gx - x
                dy = gy - y
                distance = math.hypot(dx, dy)

                if distance > self.dist_thresh:
                    cmd.speed = min(self.Kp_speed * distance, 0.2)
                    cmd.steering_angle = 0.0
                    rospy.loginfo_throttle(1.0, f"[Final Approach] dist={distance:.2f}")
                else:
                    rospy.loginfo("Final position locked. Done...")
                    self.state = "do_parking"

            elif self.state == "reverse_align":
                if abs(final_yaw_error) > self.yaw_thresh:
                    cmd.speed = -0.3
                    cmd.steering_angle = np.clip(-self.Kp_steer * final_yaw_error, -30.0, 30.0)
                    rospy.loginfo_throttle(1.0, f"[Reverse Align] yaw_err={math.degrees(final_yaw_error):.1f}°")
                else:
                    rospy.loginfo("Heading aligned in reverse. Preparing to park...")
                    self.state = "do_parking"

            elif self.state == "do_parking":
                self.trigger_parking()

            elif self.state == "done":
                cmd.speed = 0.0
                cmd.steering_angle = 0.0

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

    def trigger_parking(self):
        if self.mode == "parallel":
            rospy.loginfo("Executing SimpleParallelPark")
            SimpleParallelPark()
        elif self.mode == "diagonal":
            rospy.loginfo("Executing SimpleDiagonalPark")
            SimpleDiagonalPark()
        else:
            rospy.logwarn("Unknown parking mode. Skipping.")
        self.state = "done"

if __name__ == "__main__":
    try:
        AlignAndPark()
    except rospy.ROSInterruptException:
        pass

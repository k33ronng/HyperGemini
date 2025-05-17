#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import euler_from_quaternion
from parallel_func import SimpleParallelPark
from diagonal_func import SimpleDiagonalPark 

class ObstacleAwareParking:
    def __init__(self):
        rospy.init_node("obstacle_aware_parking")

        self.Kp_steer = 1.5
        self.Kp_speed = 0.5
        self.dist_thresh = 0.7
        self.yaw_thresh = 0.1

        self.pose = None
        self.goal = None
        self.state = "idle"
        self.obstacle_detected = False
        self.rate = rospy.Rate(10)
        self.parking_mode = "parallel"
        self.offset_a = rospy.get_param("~offset_a", 0)
        self.offset_b = rospy.get_param("~offset_b", 0)
        self.align_radius = rospy.get_param("~align_radius", 3.0)

        self.goal = None
        self.preparking_goal = None

        self.cmd_pub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size=10)
        rospy.Subscriber("/gem/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.lidar_callback)

        rospy.loginfo("🚗 Waiting for odometry, RViz goal, and LiDAR...")
        self.run()

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
        self.pose = (pos.x, pos.y, yaw)

    def goal_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
        self.goal = (pos.x, pos.y, yaw)

        a, b = self.offset_a, self.offset_b
        gx, gy = pos.x, pos.y

        offset_x = gx - (a * math.cos(yaw) - b * math.sin(yaw))
        offset_y = gy - (a * math.sin(yaw) + b * math.cos(yaw))

        self.preparking_goal = (offset_x, offset_y, yaw)
        self.state = "approach"

        rospy.loginfo(f"🟢 Parking goal set at: x={gx:.2f}, y={gy:.2f}, yaw={math.degrees(yaw):.1f}°")
        rospy.loginfo(f"🟡 Approaching offset pose at: x={offset_x:.2f}, y={offset_y:.2f}")

    def lidar_callback(self, msg):
        danger_zone = False
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, _ = p
            if 0.5 < x < 2.0 and abs(y) < 0.75:
                danger_zone = True
                break
        self.obstacle_detected = danger_zone

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle
    
    def trigger_parking(self):
        if self.parking_mode == "parallel":
            rospy.loginfo(" Executing SimpleParallelPark")
            SimpleParallelPark().run()
        elif self.parking_mode == "diagonal":
            rospy.loginfo(" Executing SimpleDiagonalPark")
            SimpleDiagonalPark().run()
        else:
            rospy.logwarn(" Unknown parking mode. Skipping.")
        self.state = "done"

    def run(self):
        while not rospy.is_shutdown():
            if self.pose is None:
                rospy.loginfo_throttle(2.0, " Waiting for odometry...")
                self.rate.sleep()
                continue

            if self.goal is None or self.preparking_goal is None:
                rospy.loginfo_throttle(2.0, " Click a goal in RViz to begin parking.")
                self.rate.sleep()
                continue

            if self.obstacle_detected:
                rospy.logwarn_throttle(1.0, " Obstacle detected! Pausing movement.")
                self.cmd_pub.publish(AckermannDrive())
                self.rate.sleep()
                continue

            x, y, yaw = self.pose

            if self.state in ["approach", "reverse_approach"]:
                gx, gy, gyaw = self.preparking_goal
            else:
                gx, gy, gyaw = self.goal

            dx = gx - x
            dy = gy - y
            distance = math.hypot(dx, dy)
            heading_to_goal = math.atan2(dy, dx)
            heading_error = self.normalize_angle(heading_to_goal - yaw)
            final_yaw_error = self.normalize_angle(gyaw - yaw)

            cmd = AckermannDrive()

            if self.state == "approach":
                if abs(heading_error) > math.pi / 2 and distance > 2.0:
                    rospy.loginfo("Goal is behind. Switching to reverse.")
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
                    rospy.loginfo("Reached offset. Aligning heading...")
                    self.state = "align"


            elif self.state == "reverse_approach":
                if distance > self.dist_thresh:
                    adjusted_heading = self.normalize_angle(heading_error + math.pi)
                    cmd.speed = -min(self.Kp_speed * distance, 0.5)
                    cmd.steering_angle = np.clip(-self.Kp_steer * adjusted_heading, -30.0, 30.0)
                    rospy.loginfo_throttle(1.0, f"[Reverse] dist={distance:.2f}, reverse_yaw_err={math.degrees(adjusted_heading):.1f}°")
                else:
                    rospy.loginfo("Reached offset in reverse. Aligning heading...")
                    self.state = "reverse_align"

            elif self.state == "align":
                if abs(final_yaw_error) > self.yaw_thresh:
                    cmd.speed = 0.2
                    cmd.steering_angle = np.clip(self.Kp_steer * final_yaw_error, -30.0, 30.0)
                    rospy.loginfo_throttle(1.0, f"[Align] yaw_err={math.degrees(final_yaw_error):.1f}°")
                else:
                    rospy.loginfo("Heading aligned. Fine-tuning position...")
                    self.state = "final_approach"

            elif self.state == "final_approach":
                gx, gy, _ = self.preparking_goal
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
                    rospy.loginfo("Heading aligned in reverse. Done...")
                    self.state = "do_parking"

            elif self.state == "do_parking":
                self.state = "done"

            elif self.state == "done":
                cmd = AckermannDrive()

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

    def trigger_parking(self):
        if self.parking_mode == "parallel":
            rospy.loginfo(" Executing SimpleParallelPark")
            SimpleParallelPark().run()
        elif self.parking_mode == "diagonal":
            rospy.loginfo(" Executing SimpleDiagonalPark")
            SimpleDiagonalPark().run()
        else:
            rospy.logwarn(" Unknown parking mode. Skipping.")
        self.state = "done"
if __name__ == "__main__":
    try:
        ObstacleAwareParking()
    except rospy.ROSInterruptException:
        pass

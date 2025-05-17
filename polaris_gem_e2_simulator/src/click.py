#!/usr/bin/env python3
import rospy
import math
import reeds_shepp
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ClickReedsSheppPlanner:
    def __init__(self):
        rospy.init_node("click_reeds_shepp_planner")

        self.turning_radius = rospy.get_param("~turning_radius", 3.0)
        self.step_size = rospy.get_param("~step_size", 0.2)
        self.current_pose = None

        rospy.Subscriber("/gem/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=1, latch=True)

        rospy.loginfo("🖱️ Click a goal in RViz to generate Reeds-Shepp path...")
        rospy.spin()


    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
        self.current_pose = (pos.x, pos.y, yaw)

    def goal_callback(self, msg):
        if self.current_pose is None:
            rospy.logwarn("⚠️ No odometry data yet. Cannot plan path.")
            return

        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gq = msg.pose.orientation
        gyaw = euler_from_quaternion([gq.x, gq.y, gq.z, gq.w])[2]

        start = self.current_pose
        goal = (gx, gy, gyaw)

        path_points = reeds_shepp.path_sample(start, goal, self.turning_radius, self.step_size)

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"  # or "odom", depending on your setup

        for pt in path_points:
            x, y, yaw, *_ = pt  # ✅ Only extract what you need
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation = Quaternion(*q)
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo("✅ Reeds–Shepp path published to /planned_path")


if __name__ == "__main__":
    try:
        ClickReedsSheppPlanner()
    except rospy.ROSInterruptException:
        pass

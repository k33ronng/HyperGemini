#!/usr/bin/env python3
import rospy
import math
import reeds_shepp
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry

class ClickReedsSheppPlanner:
    def __init__(self):
        rospy.init_node("click_reeds_shepp_planner")
        self.turning_radius = rospy.get_param("~turning_radius", 4.0)

        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=1)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        rospy.Subscriber("/gem/odom", Odometry, self.odom_callback)

        self.current_pose = None
        rospy.loginfo("🟢 Ready: Click a goal in RViz.")
        rospy.spin()

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
        self.current_pose = (pos.x, pos.y, yaw)

    def goal_callback(self, msg):
        if self.current_pose is None:
            rospy.logwarn("⚠️ Odometry not yet received.")
            return

        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gq = msg.pose.orientation
        gyaw = euler_from_quaternion([gq.x, gq.y, gq.z, gq.w])[2]

        start = self.current_pose
        goal = (gx, gy, gyaw)

        # Generate Reeds–Shepp path
        try:
            path_points = reeds_shepp.path_sample(start, goal, self.turning_radius, 0.05)
        except Exception as e:
            rospy.logerr(f"❌ Failed to generate path: {e}")
            return

        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = rospy.Time.now()

        for pt in path_points:
            x, y, yaw = pt[:3]
            direction = pt[3] if len(pt) > 3 else 1.0  # Fallback to forward

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = Point(x, y, 0.0)

            # Store direction in `orientation.w` (not normally used)
            q = quaternion_from_euler(0, 0, yaw)
            q = list(q)
            q[3] = direction  # Hack: replace w with direction
            pose.pose.orientation = Quaternion(*q)

            path_msg.poses.append(pose)


        rospy.loginfo(f"✅ Published Reeds–Shepp path with {len(path_points)} poses.")
        self.path_pub.publish(path_msg)

if __name__ == "__main__":
    try:
        ClickReedsSheppPlanner()
    except rospy.ROSInterruptException:
        pass

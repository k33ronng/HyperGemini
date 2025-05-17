#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
import tf

class GazeboOdomPublisher:
    def __init__(self):
        rospy.init_node("gazebo_odom_publisher")

        self.odom_pub = rospy.Publisher("/gem/odom", Odometry, queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

        self.model_name = rospy.get_param("~model_name", "gem")  # try "gem", or set manually
        self.model_index = None

        rospy.loginfo("Waiting for model_states to include '{}'...".format(self.model_name))
        rospy.spin()

    def model_states_callback(self, msg):
        # Find index once
        if self.model_index is None:
            if self.model_name in msg.name:
                self.model_index = msg.name.index(self.model_name)
                rospy.loginfo("Found model '{}' at index {}".format(self.model_name, self.model_index))
            else:
                rospy.logwarn_throttle(2.0, "Model '{}' not found in model_states.".format(self.model_name))
                return

        pose = msg.pose[self.model_index]
        twist = msg.twist[self.model_index]

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose = pose
        odom.twist.twist = twist

        self.odom_pub.publish(odom)

if __name__ == "__main__":
    try:
        GazeboOdomPublisher()
    except rospy.ROSInterruptException:
        pass


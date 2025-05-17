#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry

class OdomTFBroadcaster:
    def __init__(self):
        rospy.init_node("odom_tf_broadcaster")
        self.br = tf.TransformBroadcaster()
        rospy.Subscriber("/gem/odom", Odometry, self.odom_callback)
        rospy.spin()

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.br.sendTransform(
            (position.x, position.y, 0),
            (orientation.x, orientation.y, orientation.z, orientation.w),
            rospy.Time.now(),
            "base_link",
            "odom"
        )

if __name__ == "__main__":
    try:
        OdomTFBroadcaster()
    except rospy.ROSInterruptException:
        pass

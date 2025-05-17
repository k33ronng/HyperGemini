#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDrive


class SimpleParallelPark:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size=10)
        # self.cmd_pub = rospy.Publisher("/gem/stanley_gnss_cmd", AckermannDrive, queue_size=10)

        self.rate = rospy.Rate(10)

        while self.cmd_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logwarn("Waiting for /ackermann_cmd connection...")
            rospy.sleep(0.1)

    def send_cmd(self, steer_deg, speed, duration):
        cmd = AckermannDrive()
        cmd.steering_angle = steer_deg
        cmd.speed = speed
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

    def stop(self):
        self.cmd_pub.publish(AckermannDrive())
        rospy.sleep(0.5)

    def run(self):
        max_steer = 35.0
        reverse_speed = -0.5
        forward_speed = 0.5

        rospy.loginfo("Step 1: Turn right and reverse")
        self.send_cmd(steer_deg= -max_steer, speed=reverse_speed, duration=6.0)

        # Step 2: Straighten wheels, reverse
        rospy.loginfo("Step 2: Straight stop")
        self.send_cmd(steer_deg=0.0, speed=0, duration=2.5)
                # Step 2: Straighten wheels, reverse
        rospy.loginfo("Step 3: Straight reverse") # change the duration based on the distance to the right obstacle
        self.send_cmd(steer_deg=0.0, speed=reverse_speed, duration=2.5)

        rospy.loginfo("Step 4: Straight stop")
        self.send_cmd(steer_deg=0.0, speed=0, duration=2.5)
        # Step 3: Full steer left, reverse
        rospy.loginfo("Step 5: Turn left and reverse")
        self.send_cmd(steer_deg=max_steer, speed=reverse_speed, duration=6.0)

        rospy.loginfo("Step 6: go forward")
        self.send_cmd(steer_deg=0, speed=forward_speed, duration=2.0)

        rospy.loginfo("✅ Parallel parking complete.")
        self.stop()

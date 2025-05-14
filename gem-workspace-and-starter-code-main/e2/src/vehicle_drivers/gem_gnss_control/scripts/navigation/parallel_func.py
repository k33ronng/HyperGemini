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
        reverse_speed = -1
        forward_speed = 1

        rospy.loginfo("Step 1: Turn right and reverse")
        self.send_cmd(-max_steer, forward_speed, 3.0)

        rospy.loginfo("Step 2: Straight stop")
        self.send_cmd(0.0, 0, 1)

        rospy.loginfo("Step 3: Straight forward")
        self.send_cmd(0.0, forward_speed, 5)

        rospy.loginfo("Step 4: Straight stop")
        self.send_cmd(0.0, 0, 1)

        rospy.loginfo("Step 5: Straight reverse")
        self.send_cmd(0.0, reverse_speed, 5)

        rospy.loginfo("Step 6: Straight stop")
        self.send_cmd(0.0, 0, 1)

        rospy.loginfo("Step 7: Turn left and reverse")
        self.send_cmd(-max_steer, reverse_speed, 3.0)

        rospy.loginfo("Step 8: Go forward")
        self.send_cmd(0.0, forward_speed, 1.0)

        rospy.loginfo("✅ Parallel parking complete.")
        self.stop()

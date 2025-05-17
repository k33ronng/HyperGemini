import rospy
from ackermann_msgs.msg import AckermannDrive

class SimpleDiagonalPark:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size=10)
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

    def stop(self, pause_time=1.0):
        self.cmd_pub.publish(AckermannDrive())
        rospy.sleep(pause_time)

    def run(self):
        max_steer = 35.0
        reverse_speed = -1.0
        forward_speed = 1.0

        rospy.loginfo("[Step 0] Turn right and stay")
        self.send_cmd(steer_deg=-max_steer, speed=0, duration=1.0)

        rospy.loginfo("[Step 1] Turn right and move forward")
        self.send_cmd(steer_deg=-max_steer, speed=forward_speed, duration=3.0)

        rospy.loginfo("[Step 2] Stop")
        self.send_cmd(steer_deg=0.0, speed=0.0, duration=1.0)

        rospy.loginfo("[Step 3] Move straight forward")
        self.send_cmd(steer_deg=0.0, speed=forward_speed, duration=5.0)

        rospy.loginfo("[Step 4] Stop")
        self.send_cmd(steer_deg=0.0, speed=0.0, duration=1.0)

        rospy.loginfo("[Step 5] Reverse straight")
        self.send_cmd(steer_deg=0.0, speed=reverse_speed, duration=8.0)

        rospy.loginfo("[Step 6] Stop")
        self.send_cmd(steer_deg=-max_steer, speed=0.0, duration=1.0)

        rospy.loginfo("[Step 7] Turn left and reverse")
        self.send_cmd(steer_deg=-max_steer, speed=reverse_speed, duration=2.5)

        rospy.loginfo("[Step 8] Move forward")
        self.send_cmd(steer_deg=0.0, speed=forward_speed, duration=1.0)

        rospy.loginfo("✅ Diagonal parking complete. Vehicle stopped.")
        self.stop()

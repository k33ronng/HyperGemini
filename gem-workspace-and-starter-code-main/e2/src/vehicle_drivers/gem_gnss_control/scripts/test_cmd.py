#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from pacmod_msgs.msg import PacmodCmd

def main():
    rospy.init_node("test_reverse_command")

    enable_pub = rospy.Publisher("/pacmod/as_rx/enable", Bool, queue_size=1, latch=True)
    gear_pub   = rospy.Publisher("/pacmod/as_rx/shift_cmd", PacmodCmd, queue_size=1, latch=True)
    accel_pub  = rospy.Publisher("/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size=1)
    brake_pub  = rospy.Publisher("/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size=1, latch=True)

    rospy.sleep(1.0)  # Let publishers connect

    rospy.loginfo("🔧 Enabling PACMod control")
    enable_pub.publish(Bool(data=True))

    rospy.sleep(0.5)

    rospy.loginfo("⚙️  Setting gear to REVERSE (1)")
    gear_pub.publish(PacmodCmd(ui16_cmd=1))  # Reverse

    rospy.sleep(0.5)

    rospy.loginfo("🛑 Releasing brakes")
    brake_pub.publish(PacmodCmd(f64_cmd=0.0))

    rospy.sleep(0.5)

    rospy.loginfo("🚗 Sending reverse acceleration = 1.0")
    accel_cmd = PacmodCmd(f64_cmd=0.5)
    rate = rospy.Rate(10)
    for _ in range(20):  # 2 seconds of command
        accel_pub.publish(accel_cmd)
        rate.sleep()

    rospy.loginfo("✅ Done sending reverse command")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

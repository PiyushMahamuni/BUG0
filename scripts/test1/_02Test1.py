#!/usr/bin/env python
# make the robot rotate
import rospy
from geometry_msgs.msg import Twist
from math import pi

# CONSTATNS
NODE_NAME = "Test2"
VEL_TOPIC = "cmd_vel"

# GLOBALS
vel_pub: rospy.Publisher = None
vel_cmd = Twist()


def setup():
    rospy.init_node(NODE_NAME)
    global vel_pub
    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
    vel_cmd.angular.z = 10 * pi / 180


def main():
    setup()

    loop_rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        vel_pub.publish(vel_cmd)
        try:
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    main()
#! /usr/bin/env python
# figuring out how laser scanner data is published
# run alogn with roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch and observe output
# by placing obstacles in simulations around the robot.
import rospy
from sensor_msgs.msg import LaserScan
from math import degrees

# CONSTANTS
NODE_NAME = "Test1"
SCAN_TOPIC = "scan"

# GLOBALS
scan_sub = None

def clbk_scan(msg: LaserScan):
    min_range_ind = 0
    min_range = msg.ranges[min_range_ind]
    for i, r in enumerate(msg.ranges):
        if r < min_range:
            min_range = r
            min_range_ind = i
    angle = msg.angle_min + msg.angle_increment * min_range_ind
    rospy.loginfo(f"Obstacle detected at angle {degrees(angle)}")


def setup():
    rospy.init_node(NODE_NAME)
    global scan_sub
    scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, clbk_scan, queue_size=1)


def main():
    setup()
    rospy.spin()


if __name__ == "__main__":
    main()
#!/usr/bin/env python
#See if given region is free or not
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from math import pi, radians

# CONSTANTS
NODE_NAME = "_01Test2"
LOOK_ANGLE_TOPIC = "look_angle"
SCAN_TOPIC = "scan"
DTOT = 2 # DISTANCE TO OBSTACLE TRESHOLD, 2 m
FIELD_OF_VIS = pi / 2 # 90 degrees

# GOLBALS
look_sub = None
look_angle = 0
scan_sub = None


def map0to2pi(angle: float) -> float:
    """maps given angle to corresponding angle in 0 to 2pi value"""
    pi_2 = 2 * pi
    while angle > pi_2:
        angle -= pi_2
    while angle < 0:
        angle = pi_2 - angle
    return angle


# LOOK_ANGLE_TOPIC clkb
def clbk_look_angle(msg: Float32) -> None:
    """updates look_angle global variable"""
    global look_angle
    look_angle = map0to2pi(radians(msg.data))


# SCAN TOPIC clbk
def clbk_scan(msg: LaserScan) -> None:
    """ checks if the 90 degree region around the look_angle is free """
    d = FIELD_OF_VIS / 2
    start_ind = int((map0to2pi(look_angle - d) - msg.angle_min) / msg.angle_increment)
    end_ind = int((map0to2pi(look_angle + d) - msg.angle_min) / msg.angle_increment)
    is_clear = True
    if start_ind > end_ind:
        for r in msg.ranges[start_ind:]:
            if r < DTOT:
                is_clear = False
                break
        if is_clear:
            for r in msg.ranges[:end_ind]:
                if r < DTOT:
                    is_clear = False
                    break
    else:
        for r in msg.ranges[start_ind:end_ind]:
            if r < DTOT:
                is_clear = False
                break
    rospy.loginfo(f"look_angle: {look_angle} | is_clear: {is_clear}\nstart_ind: {start_ind} | end_ind: {end_ind}\n")


def setup():
    rospy.init_node(NODE_NAME)
    global look_sub, scan_sub
    look_sub = rospy.Subscriber(LOOK_ANGLE_TOPIC, Float32, clbk_look_angle, queue_size=1)
    scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, clbk_scan, queue_size=1)


def main():
    setup()
    rospy.spin()


if __name__ == "__main__":
    main()
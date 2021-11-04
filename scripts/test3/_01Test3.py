#!/usr/bin/env python

# go to goal

import rospy
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from geometry_msgs.msg import Twist, Point32
import tf
from math import atan2, pi, sqrt
from bug0.srv import UpdateGoalRequest, UpdateGoalResponse, UpdateGoal
from bug0.msg import NavError
from sys import exit


# CONSTANTS
NODE_NAME = "gtg_server"
GTG_SRV = "go_to_goal"
UG_SRV = "update_goal"
VEL_TOPIC = "cmd_vel"
ERR_TOPIC = "nav_error"
WORLD_FRAME = "odom"
ROBOT_FRAME = "base_footprint"
GOAL = Point32()
LIN_VEL = 0.10
ANG_VEL = 20 * pi / 180
ANG_THRESH = 2 * pi / 180 # 2 degrees
LIN_THRESH = 0.07 # 7 cm

# GLOBALS
gtg_srvr = None # go to goal server
ug_srvr = None  # update goal server
active = False
vel_pub = None
vel_cmd = Twist()
odom_sub = None
tf_listener = None

nav_err = NavError()
nav_err_pub = None
to_correct_dist = False

def stop():
    vel_cmd.linear.x = vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)
    rospy.loginfo("\n --- STOPPING ROBOT ---\n")


def map0to2pi(angle: float) -> float:
    """"maps given angle to corresponding value from range 0 to 2 pi"""
    pi_2 = 2 * pi
    while angle > pi_2:
        angle -= pi_2
    while angle < 0:
        angle = pi_2 - angle
    return angle


def normalize_angle(angle: float) -> float:
    """maps given angle to corresponding value from range -pi to pi"""
    pi_2 = 2 * pi
    while angle > pi:
        angle -= pi_2
    while angle < -pi:
        angle += pi_2
    return angle


def update_pose() -> None:
    # TODO: remove following line
    # tf_listener = tf.TransformListener()
    translation, orientation = tf_listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, rospy.Time())
    if translation:
        _, _, yaw = tf.transformations.euler_from_quaternion(orientation)
        dx = GOAL.x - translation[0]
        dy = GOAL.y - translation[1]
        des_yaw = atan2(dy, dx)
        nav_err.yaw_error = normalize_angle(des_yaw - yaw)
        # TODO: replace normalize_angle() with map0to2pi()
        nav_err.dist_error = sqrt(dy * dy + dx * dx)
        nav_err_pub.publish(nav_err)


def correct_yaw():
    """Corrects yaw error"""
    dyaw = normalize_angle(nav_err.yaw_error)
    vel_cmd.angular.z = -ANG_VEL if dyaw < -ANG_THRESH else (ANG_VEL if dyaw > ANG_THRESH else 0)
    global to_correct_dist
    # start correcting for distance error only when angular error is Within limits
    if -ANG_THRESH < dyaw < ANG_THRESH:
        to_correct_dist = True
    else:
        to_correct_dist = False
        vel_cmd.linear.x = 0


def correct_dist():
    """corrects distance error """
    if nav_err.dist_error < LIN_THRESH:
        rospy.loginfo(f"Reached Goal x: {GOAL.x} y: {GOAL.y}")
        stop()
        global active
        active = False
        GOAL.x = GOAL.y = 0
        return None
    vel_cmd.linear.x = LIN_VEL


def ug_handler(req: UpdateGoalRequest) -> UpdateGoalRequest:
    GOAL.x = req.goal.x
    GOAL.y = req.goal.y
    GOAL.z = req.goal.z
    rospy.loginfo(f"\n---- GOAL UPDATED: x: {GOAL.x} y: {GOAL.y} ----\n")
    return UpdateGoalResponse(success=True)


def gtg_handler(req: SetBoolRequest) -> SetBoolResponse:
    global active
    if req.data and active:
        rospy.loginfo("gtg server already active!")
    elif req.data:
        rospy.loginfo("\n---- gtg server now active! ----\n")
        active = req.data
        stop()
    elif not req.data and not active:
        rospy.loginfo("gtg server already inactive!")
    else:
        rospy.loginfo("\n---- gtg server now inactive ----\n")
        active = req.data
        stop()
    return SetBoolResponse(success=True, message="Done!")


def setup():
    rospy.init_node(NODE_NAME)
    global gtg_srvr, ug_srvr, vel_pub, odom_sub, nav_err_pub, tf_listener
    gtg_srvr = rospy.Service(GTG_SRV, SetBool, gtg_handler)
    ug_srvr = rospy.Service(UG_SRV, UpdateGoal, ug_handler)
    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
    tf_listener = tf.TransformListener()
    nav_err_pub = rospy.Publisher(ERR_TOPIC, NavError, queue_size=1)
    try:
       tf_listener.waitForTransform(ROBOT_FRAME, WORLD_FRAME, rospy.Time(), rospy.Duration(10))
    except rospy.exceptions.ROSInterruptException as e:
        exit(0)


def main():
    setup()

    loop_rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        while not rospy.is_shutdown() and active:
            update_pose()
            correct_yaw()
            if to_correct_dist:
                correct_dist()
            vel_pub.publish(vel_cmd)
            try:
                loop_rate.sleep()
            except rospy.ROSInterruptException:
                pass


if __name__ == "__main__":
    main()
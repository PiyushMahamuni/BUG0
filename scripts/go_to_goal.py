#!/usr/bin/env python

# go to goal

import rospy
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from geometry_msgs.msg import Twist, Point32
import tf
from math import atan2, pi, radians, sqrt, fabs
from bug0.srv import UpdateGoalRequest, UpdateGoalResponse, UpdateGoal, SetVelResponse, SetVelRequest, SetVel
from bug0.msg import NavError
from sys import exit


# CONSTANTS
NODE_NAME = "gtg_server"
# services
GTG_SRV = "go_to_goal"
UG_SRV = "update_goal"
SET_VEL_SRV = "set_vel"
# topics
VEL_TOPIC = "cmd_vel"
ERR_TOPIC = "nav_error"
# frames
WORLD_FRAME = "odom"
ROBOT_FRAME = "base_footprint"
# navigation
GOAL = Point32()
ANG_THRESH = 3 * pi / 180 # 2 degrees
LIN_THRESH = 0.03 # 7 cm

# GLOBALS
# servers
gtg_srvr = None # go to goal server
ug_srvr = None  # update goal server
set_vel_srvr = None
# publishers
vel_pub = None
nav_err_pub = None
# subsribers / action clients
tf_listener = None
# messages
vel_cmd = Twist()
nav_err = NavError()
# navigation
cruising_vel = SetVelRequest()
cruising_vel.lin_vel = 0.20 # m/s
cruising_vel.ang_vel = 30 * pi / 180 # 30 deg/s
approaching_vel = SetVelRequest()
approaching_vel.lin_vel = cruising_vel.lin_vel * 0.5
approaching_vel.ang_vel = cruising_vel.ang_vel * 0.5
# booleans
active = False


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


def update_error() -> None:
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


def correct_error():
    """Corrects yaw error"""
    if nav_err.dist_error < LIN_THRESH:
        rospy.loginfo("\n---- Reached Goal! x: {GOAL.x}, y: {GOAL.y} ----\n")
        stop()
        global active
        active = False
    dyaw = normalize_angle(nav_err.yaw_error)
    vel_cmd.angular.z = cruising_vel.ang_vel = cruising_vel.ang_vel * -1 if (cruising_vel.ang_vel > 0 and dyaw < 0) or (cruising_vel.ang_vel < 0 and dyaw > 0) else cruising_vel.ang_vel
    
    # start correcting for distance error only when angular error is Within limits
    if -ANG_THRESH < dyaw < ANG_THRESH:
        vel_cmd.angular.z = 0
        if nav_err.dist_error < 0.3:
            vel_cmd.linear.x = vel_cmd.linear.x * 0.95 + 0.05 * approaching_vel.lin_vel
        else:
            vel_cmd.linear.x = cruising_vel.lin_vel
    # slow down if error is about to diminish
    elif fabs(dyaw) < radians(15):
        vel_cmd.angular.z = 0.95 * vel_cmd.angular.z + 0.05 * approaching_vel.ang_vel
    else:
        vel_cmd.linear.x = 0


def reset() -> None:
    """resets the velocity controller
    * Doesn't change maximum velocities and gains"""
    stop()
    global active
    active = False


# handlers
def set_vel_handler(req: SetVelRequest) -> SetVelResponse:
    """sets the linear and angular velocities at which the robot will chase goal location"""
    global lin_vel, ang_vel
    lin_vel = req.lin_vel
    ang_vel = req.ang_vel
    return SetVelResponse(success=True)


def ug_handler(req: UpdateGoalRequest) -> UpdateGoalRequest:
    reset()
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
        reset()
        rospy.loginfo("\n---- gtg server now active! ----\n")
        active = req.data
    elif not req.data and not active:
        rospy.loginfo("gtg server already inactive!")
    else:
        rospy.loginfo("\n---- gtg server now inactive ----\n")
        active = req.data
        stop()
    return SetBoolResponse(success=True, message="Done!")


# setting up ros node
def setup():
    rospy.init_node(NODE_NAME)
    global gtg_srvr, ug_srvr, vel_pub, odom_sub, nav_err_pub, tf_listener, set_max_vel_srvr, set_gains_srvr
    gtg_srvr = rospy.Service(GTG_SRV, SetBool, gtg_handler)
    ug_srvr = rospy.Service(UG_SRV, UpdateGoal, ug_handler)
    set_max_vel_srvr = rospy.Service(SET_VEL_SRV, SetVel, set_vel_handler)
    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
    tf_listener = tf.TransformListener()
    nav_err_pub = rospy.Publisher(ERR_TOPIC, NavError, queue_size=1)
    try:
       tf_listener.waitForTransform(ROBOT_FRAME, WORLD_FRAME, rospy.Time(), rospy.Duration(10))
    except rospy.exceptions.ROSInterruptException as e:
        exit(0)


def main():
    setup()

    loop_rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        update_error()
        if active:
            correct_error()
            vel_pub.publish(vel_cmd)
        try:
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    main()
#!/usr/bin/env python

# main driver node
import rospy
from std_srvs.srv import SetBool, SetBoolRequest
from bug0.srv import UpdateGoal, UpdateGoalRequest
from bug0.msg import NavError
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan
from math import pi, cos, sin, fabs

# CONSTANTS
NODE_NAME = "main_driver"
# topics
SCAN_TOPIC = "scan"
ERR_TOPIC = "nav_error"
# services
FOLLOW_WALL_SRV = "fw/activate"
GO_TO_GOAL_SRV = "gtg/activate"
UPDATE_GOAL_SRV = "update_goal"
# navigation
DTOT = 0.4 # m
SIDE_CLEARANCE = 0.20
MAX_CLEARANCE = 0.7
LIN_THRSH = 0.04 # 4 cm

# GLOBALS
reached_goal: bool = False
following_wall: bool = False
going_to_goal: bool = False
# services
follow_wall: rospy.ServiceProxy = None
go_to_goal: rospy.ServiceProxy = None
update_goal: rospy.ServiceProxy = None
# navigation
goal = Point32()
yaw_err: float = None
front_clearance: float = None
# subscribers
scan_sub: rospy.Subscriber = None
err_sub: rospy.Subscriber = None


def clbk_err(msg: NavError) -> None:
    global yaw_err
    yaw_err = msg.yaw_error
    global reached_goal, front_clearance
    reached_goal = msg.dist_error < LIN_THRSH
    front_clearance = msg.dist_error if msg.dist_error < MAX_CLEARANCE else MAX_CLEARANCE


def map0to2pi(angle: float) -> float:
    """maps given angles in radians to corresponding angle in range from 0 to 2pi"""
    pi_2 = pi * 2
    while angle > pi_2:
        angle -= pi_2
    while angle < 0:
        angle += pi_2
    return angle


def look_for_obstacles(angle: float, scan: LaserScan, fov: float=pi) -> bool:
    f"""Returns False if any obstalces closer are found in the given direction"""
    fov /= 2
    end_ind = int(map0to2pi(angle + fov - scan.angle_min) / scan.angle_increment)
    start_ind = int(map0to2pi(angle - fov - scan.angle_min) / scan.angle_increment)
    angle = map0to2pi(-fov)
    if start_ind > end_ind:
        for r in scan.ranges[start_ind:]:
            x = r * cos(angle)
            y = fabs(r * sin(angle))
            if x < front_clearance and y < SIDE_CLEARANCE:
                return False
            angle += scan.angle_increment
        for r in scan.ranges[:end_ind]:
            x = r * cos(angle)
            y = fabs(r * sin(angle))
            if x < front_clearance and y < SIDE_CLEARANCE:
                return False
            angle += scan.angle_increment
    for r in scan.ranges[start_ind:end_ind]:
        x = r * cos(angle)
        y = fabs(r * sin(angle))
        if x < front_clearance and y < SIDE_CLEARANCE:
            return False
        angle += scan.angle_increment
    return True


def clbk_scan(scan: LaserScan) -> None:
    if not reached_goal:
        clear = look_for_obstacles(yaw_err, scan)
        global following_wall, going_to_goal
        if clear and following_wall:
            follow_wall.call(SetBoolRequest(data=False))
            go_to_goal.call(SetBoolRequest(data=True))
            going_to_goal = True
            following_wall = False
        elif not clear and going_to_goal:
            go_to_goal.call(SetBoolRequest(data=False))
            follow_wall.call(SetBoolRequest(data=True))
            going_to_goal = False
            following_wall = True
    

def setup() -> None:
    rospy.init_node(NODE_NAME)
    # setting up goal location
    goal.x = float(rospy.get_param("~x_goal"))
    goal.y = float(rospy.get_param("~y_goal"))
    # setting up service clients
    global follow_wall, go_to_goal, update_goal
    rospy.wait_for_service(FOLLOW_WALL_SRV)
    follow_wall = rospy.ServiceProxy(FOLLOW_WALL_SRV, SetBool)
    rospy.wait_for_service(GO_TO_GOAL_SRV)
    go_to_goal = rospy.ServiceProxy(GO_TO_GOAL_SRV, SetBool)
    rospy.wait_for_service(UPDATE_GOAL_SRV)
    update_goal = rospy.ServiceProxy(UPDATE_GOAL_SRV, UpdateGoal)
    # setting up subscribers
    global scan_sub, err_sub
    err_sub = rospy.Subscriber(ERR_TOPIC, NavError, clbk_err, queue_size=1)
    rospy.wait_for_message(ERR_TOPIC, NavError)
    scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, clbk_scan, queue_size=1)


def main():
    setup()
    update_goal.call(UpdateGoalRequest(goal=goal))
    rospy.sleep(5)
    go_to_goal.call(SetBoolRequest(data=True))
    global going_to_goal
    going_to_goal = True
    rospy.loginfo(f"\n---- {NODE_NAME} STARTING TO MOVE THE BOT TO :x= {goal.x} y={goal.y} ----\n")
    rospy.spin()


if __name__ == "__main__":
    main()

#! /usr/bin/env python
# 1. check if you have lost the wall
# 2. Find the wall and turn left
# 3. Follow the wall and if right region is free, turn right
# 4. loop

import rospy
from bug0.srv import SetVel, SetVelRequest, SetVelResponse
from math import pi, fabs, sin, cos, radians
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

# CONSTANTS
NODE_NAME = "follow_wall_server"
# services
FW_SRV = "fw/activate"
SV_SRV = "fw/set_vel"
# topics
SCAN_TOPIC = "scan"
VEL_TOPIC = "cmd_vel"
# navigation
SIDE_CLEARANCE = 0.18
FRONT_CLEARANCE = 0.38
OFFSET = 0.032
DIST2WALL = 0.7


# GLOBALS
# states
machine_states = {"Inactive": True, "Finding Wall": False, "Turning Left": False, "Turning Right": False, "Following Wall": False}
# servers
fw_srvr: rospy.Service = None # follow wall server
sv_srvr: rospy.Service = None # set velocity server
# navigation
scan: LaserScan = None
vel = SetVelRequest()
vel.lin_vel = 0.13
vel.ang_vel = pi / 8
vel_cmd = Twist()
nt_obstacles = {}
regions = {}
lost_wall = True
# publishers
vel_pub: rospy.Publisher = None
# subscribers
scan_sub: rospy.Subscriber = None
# rates
loop_rate: rospy.Rate = None



def activate_handler(req: SetBoolRequest) -> SetBoolResponse:
    if machine_states["Inactive"] and not req.data:
        rospy.loginfo(f"\n---- {NODE_NAME} is already inactive! ----\n")
    elif machine_states["Inactive"]:
        rospy.loginfo(f"\n---- {NODE_NAME} has now been activated! ----\n")
        machine_states["Inactive"] = False
        machine_states["Finding Wall"] =  True
    elif not req.data:
        rospy.loginfo(f"\n---- {NODE_NAME} has now been inactivated! ----\n")
        machine_states["Inactive"] = True
        machine_states["Finding Wall"] = machine_states["Turning Left"] = machine_states["Turning Right"] = machine_states["Following Wall"] = False
    else:
        rospy.loginfo(f"\n---- {NODE_NAME} is already active! ----\n")
    return SetBoolResponse(success=True, message="Done!")


def sv_handler(req: SetVelRequest) -> SetVelResponse:
    vel.lin_vel = req.lin_vel
    vel.ang_vel = req.ang_vel
    return SetVelResponse(success=True)


def map0to2pi(angle: float) -> float:
    pi_2 = pi * 2
    while angle > pi_2:
        angle -= pi_2
    while angle < 0:
        angle += pi_2
    return angle


def clbk_scan(msg: LaserScan) -> None:
    global scan
    scan = msg
    lst_wall = is_clear = nt_obst = True
    x_comp = FRONT_CLEARANCE
    y_comp = SIDE_CLEARANCE
    x_comp1 = x_comp + OFFSET
    y_comp1 = y_comp + OFFSET
    # angle = msg.angle_increment * index+ msg.angle_min
    angle = map0to2pi(-pi/4)
    start_ind = int(map0to2pi(angle - msg.angle_min) / msg.angle_increment)
    for r in msg.ranges[start_ind:]:
        x = r * cos(angle)
        y = -r * sin(angle)
        if x < x_comp and y < y_comp:
            lst_wall = is_clear = nt_obst = False
            break
        is_clear = is_clear and (x > x_comp1 or y > y_comp1)
        lst_wall = lst_wall and r > DIST2WALL
        angle += msg.angle_increment
    angle = 0
    end_ind = int(map0to2pi(pi / 4 - msg.angle_min) / msg.angle_increment)
    if lst_wall:
        for r in msg.ranges[:end_ind]:
            x = r * cos(angle)
            y = r * sin(angle)
            if x < x_comp and y < y_comp:
                lst_wall = is_clear = nt_obst = False
                break
            lst_wall = is_clear and (x > x_comp1 and y > y_comp1)
            lst_wall = lst_wall and r > DIST2WALL
            angle += msg.angle_increment
    elif is_clear:
        for r in msg.ranges[:end_ind]:
            x = r * cos(angle)
            y = r * sin(angle)
            if x < x_comp and y < y_comp:
                is_clear = nt_obst = False
                break
            is_clear = is_clear and (x > x_comp1 and y > y_comp1)
            angle += msg.angle_increment
    elif nt_obst:
        for r in msg.ranges[:end_ind]:
            if r * cos(angle) < x_comp and r * sin(angle) < y_comp:
                nt_obst = False
                break
            angle += msg.angle_increment
    nt_obstacles["front"] = nt_obst
    regions["front"] = is_clear
    start_ind = end_ind
    angle = pi / 4
    end_ind = int(map0to2pi(angle + pi / 2 - msg.angle_min) / msg.angle_increment)
    is_clear = nt_obst = True
    x_comp = SIDE_CLEARANCE
    y_comp = FRONT_CLEARANCE
    x_comp1 = x_comp + OFFSET
    y_comp1 = y_comp + OFFSET
    for r in msg.ranges[start_ind:end_ind]:
        x = fabs(r * cos(angle))
        y = r * sin(angle)
        if x < x_comp and y < y_comp:
            lst_wall = is_clear = nt_obst = False
            break
        is_clear = is_clear and (x > x_comp1 or y > y_comp1)
        lst_wall = lst_wall and r > DIST2WALL
        angle += msg.angle_increment
    nt_obstacles["left"] = nt_obst
    regions["left"] = is_clear
    start_ind = end_ind
    angle = pi / 2 + pi / 4
    end_ind = int(map0to2pi(angle + pi / 2 - msg.angle_increment) / msg.angle_increment)
    is_clear = nt_obst = True
    x_comp = FRONT_CLEARANCE
    y_comp = SIDE_CLEARANCE
    x_comp1 = x_comp + OFFSET
    y_comp1 = y_comp + OFFSET
    for r in msg.ranges[start_ind:end_ind]:
        x = -r * cos(angle)
        y = fabs(r * sin(angle))
        if x < x_comp and y < y_comp:
            lst_wall = is_clear = nt_obst = False
            break
        is_clear = is_clear and (x > x_comp1 or y > y_comp1)
        lst_wall = lst_wall and r > DIST2WALL
        angle += msg.angle_increment
    nt_obstacles["back"] = nt_obst
    regions["back"] = is_clear
    start_ind = end_ind
    angle = pi + pi / 4
    end_ind = int(map0to2pi(angle + pi / 2 - msg.angle_min) / msg.angle_increment)
    is_clear = nt_obst = True
    x_comp = SIDE_CLEARANCE
    y_comp = FRONT_CLEARANCE
    x_comp1 = x_comp + OFFSET
    y_comp1 = x_comp + OFFSET
    for r in msg.ranges[start_ind:end_ind]:
        x = fabs(r * cos(angle))
        y = -r * sin(angle)
        if x < x_comp and y < y_comp:
            lst_wall = is_clear = nt_obst = False
            break
        is_clear = is_clear and (x > x_comp1 or y > y_comp1)
        lst_wall = lst_wall and r > DIST2WALL
        angle += msg.angle_increment
    nt_obstacles["right"] = nt_obst
    regions["right"] = is_clear
    global lost_wall
    lost_wall = lst_wall


def stop_robot():
    vel_cmd.linear.x = vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)
    rospy.sleep(0.3)


def find_wall():
    vel_cmd.linear.x = vel.lin_vel
    vel_cmd.angular.z = 0
    while nt_obstacles["front"] and nt_obstacles["right"]:
        vel_pub.publish(vel_cmd)
        loop_rate.sleep()
    machine_states["Finding Wall"] = False
    if not nt_obstacles["front"]:
        machine_states["Turning Left"] = True
    else:
        machine_states["Following Wall"] = True
    stop_robot()


def turn_left():
    vel_cmd.angular.z = vel.ang_vel
    vel_cmd.linear.x = 0
    while not nt_obstacles["front"] or regions["right"]:
        vel_pub.publish(vel_cmd)
        loop_rate.sleep()
    stop_robot()
    machine_states["Turning Left"] = False
    machine_states["Following Wall"] = True


def follow_wall():
    vel_cmd.linear.x = vel.lin_vel
    while not regions["right"] and nt_obstacles["front"]:
        range1 = scan.ranges[follow_wall.ind1]
        range2 = scan.ranges[follow_wall.ind2]
        err = range2 - range1
        if range1 > DIST2WALL or range2 > DIST2WALL or fabs(err) > 0.6:
            vel_cmd.angular.z = 0
        else:
            vel_cmd.angular.z = follow_wall.gain * err
            vel_cmd.angular.z = vel.ang_vel * (1 if err > 0 else -1) if fabs(vel_cmd.angular.z) > vel.ang_vel else vel_cmd.angular.z
        vel_pub.publish(vel_cmd)
        loop_rate.sleep()
    machine_states["Following Wall"] = False
    if regions["right"]:
        machine_states["Turning Right"] = True
    else:
        machine_states["Turning Left"] = True
    stop_robot()


def turn_right():
    vel_cmd.angular.z = -vel.ang_vel
    vel_cmd.linear.x = 0
    while not nt_obstacles["front"]:
        vel_pub.publish(vel_cmd)
        loop_rate.sleep()
    OFFSET = 0.1
    while regions["front"]:
        vel_pub.publish(vel_cmd)
        loop_rate.sleep()
    OFFSET = 0.05
    stop_robot()
    machine_states["Finding Wall"] = True
    machine_states["Turning Right"] = False
    

def log_state():
    for key in machine_states:
        if machine_states[key]:
            rospy.loginfo(f"\n---- {NODE_NAME} is currently {key} ----\n")


def take_action():
    log_state()
    if machine_states["Finding Wall"]:
        find_wall()
    elif machine_states["Turning Left"]:
        turn_left()
    elif machine_states["Following Wall"]:
        follow_wall()
    elif machine_states["Turning Right"]:
        turn_right()


def setup():
    # SETTING UP NODE
    rospy.init_node(NODE_NAME)
    global fw_srvr, sv_srvr, vel_pub, scan_sub
    fw_srvr = rospy.Service(FW_SRV, SetBool, activate_handler)
    sv_srvr = rospy.Service(SV_SRV, SetVel, sv_handler)
    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
    scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, clbk_scan, queue_size=1)
    # SETTING UP RATES
    global loop_rate
    loop_rate = rospy.Rate(60)
    # SETTING UP FUNCTION ATTRIBUTES
    rospy.wait_for_message(SCAN_TOPIC, LaserScan)
    follow_wall.gain = 10
    follow_wall.ind1 = int(map0to2pi(radians(-80) - scan.angle_min) / scan.angle_increment)
    follow_wall.ind2 = int(map0to2pi(radians(-100) - scan.angle_min) / scan.angle_increment)


def main():
    setup()
    while not rospy.is_shutdown():
        if not machine_states["Inactive"]:
            take_action()
        else:
            loop_rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

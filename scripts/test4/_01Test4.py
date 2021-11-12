#! /usr/bin/env python

# testing wall following algorithms
# 1. moves forward until wall is detected
# 2. takes first turn to left to keep the wall at right, keeps moving forward
# 3. Keeps turning right or left according to scan readings from angles -60 and -120 degrees
# 4. turns right if right region is free
# 5. turns left if it is free while right and front regions are not
# 6. Keeps turning left if all three regions are blocked

import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import pi, sin, cos, fabs


# CONSTANTS
NODE_NAME = "wall_follow_server"
# topics
SCAN_TOPIC = "scan"
VEL_TOPIC = "cmd_vel"
# services
ACTIVATION_SRV = "follow_wall"
# navigation
FRONT_CLEARANCE = 0.35 # in meters
SIDE_CLEARANCE = 0.25
DTOT = 0.42
# GLOBALS
active = False
not_lost_wall = True
# navigation
regions = {'front': False, 'left': False, 'right': False}
vel_cmd = Twist()
ang_vel = pi / 6 # 30 deg/s
lin_vel = 0.25 # 27 cm/s
scan: LaserScan = None
# servers
activation_srvr: rospy.Service = None
# publishers
vel_pub: rospy.Publisher = None
# subscrivers
scan_sub: rospy.Subscriber= None
# durations
sleep: rospy.Duration = None
machine_states = {'Finding Wall': False, 'Following Wall': False, 'Turning Right': False, 'Turning Left': False, 'Inactive': True}


def stop():
    vel_cmd.linear.x = vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)
    try:
        sleep.sleep()
    except rospy.ROSInterruptException:
        return None


def map0to2pi(angle: float) -> float:
    """Returns corresponing angle mapped in range of 0 to 2pi radians for given radian angle"""
    pi_2 = pi * 2
    while angle > pi_2:
        angle -= pi_2
    while angle < 0:
        angle += pi_2
    return angle


def look_for_obstacles(dir: int, scan: LaserScan, fov: float=pi/2) -> tuple:
    """Returns true if no obstacle around 90 degrees are found to be closer than DTOT for given angle
    otherwise false. It returns false if laserscanner can't scan around given angle
    @param dir: 0=front, 1=left, 2=back, 3=right"""
    d = fov / 2 # fov = field of vision
    angle = pi * dir / 2
    start_angle = map0to2pi(angle - d)
    end_angle = map0to2pi(angle + d)
    angle = scan.angle_min
    if dir % 2:
        y_comp = SIDE_CLEARANCE
        x_comp = FRONT_CLEARANCE
    else:
        y_comp = FRONT_CLEARANCE
        x_comp = SIDE_CLEARANCE
    if scan.angle_min <= start_angle <= scan.angle_max and scan.angle_min <= end_angle <= scan.angle_max:
        start_ind = int((start_angle - scan.angle_min) / scan.angle_increment)
        end_ind = int((end_angle - scan.angle_min) / scan.angle_increment)
        is_clear = True
        nlw = False
        if start_ind > end_ind:
            for r in scan.ranges[start_ind:]:
                if fabs(r * cos(angle)) < x_comp and fabs(r * sin(angle)) < y_comp:
                    is_clear = False
                    nlw = True
                    break
                angle += scan.angle_increment
                if not nlw and r < DTOT:
                    nlw = True
            if not nlw:
                for r in scan.ranges[:end_ind]:
                    if fabs(r * cos(angle)) < x_comp and fabs(r * sin(angle)) < y_comp:
                        is_clear =  False
                        nlw = True
                        break
                    angle += scan.angle_increment
                    if not nlw and r < DTOT:
                        nlw = True
            elif is_clear:
                for r in scan.ranges[:end_ind]:
                    if fabs(r * cos(angle)) < x_comp and fabs(r * sin(angle)) < y_comp:
                        is_clear = False
                        break
                    angle += scan.angle_increment

        for r in scan.ranges[start_ind: end_ind]:
            if fabs(r* cos(angle)) < x_comp and fabs(r * sin(angle)) < y_comp:
                is_clear = False
                nlw = True
                break
            angle += scan.angle_increment
            if not nlw and r < DTOT:
                nlw = True
        return is_clear, nlw
    return False, False


def log_state() -> None:
    """Logs current status of server"""
    for key in machine_states:
        if machine_states[key]:
            rospy.loginfo(f"\n--- Server is currently {key} ----\n")


def turn_right() -> None:
    """Turns right until front region is free"""
    vel_cmd.linear.x = vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)
    loop_rate = rospy.Rate(60)
    vel_cmd.angular.z = -ang_vel
    while not regions["front"]:
        vel_pub.publish(vel_cmd)
        try:
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            return None
    while regions["front"]:
        vel_pub.publish(vel_cmd)
        try:
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            return None
    vel_cmd.linear.x = vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)
    machine_states["Turning Right"] = False
    machine_states["Turning Left"] = True


def turn_left() -> None:
    """Turns the robot left until front regions is free"""
    vel_cmd.linear.x = vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)
    loop_rate = rospy.Rate(60)
    vel_cmd.angular.z = ang_vel
    while not regions["front"] or regions["right"]:
        vel_pub.publish(vel_cmd)
        try:
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            return None
    vel_cmd.linear.x = vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)
    machine_states["Turning Left"] = False
    machine_states["Following Wall"] = True


def find_wall() -> None:
    """moves robot fwd until an obstacle is in front and then turns left until front regions is free again"""
    vel_cmd.linear.x = vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)
    loop_rate = rospy.Rate(60)
    vel_cmd.linear.x = lin_vel
    while regions["front"]:
        vel_pub.publish(vel_cmd)
        try:
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            return None
    machine_states["Turning Left"] = True
    machine_states["Finding Wall"] = False


def follow_wall():
    """Follows the wall on the right"""
    loop_rate = rospy.Rate(60)
    vel_cmd.linear.x = lin_vel
    while not regions["right"]:
        err = scan.ranges[follow_wall.ind2] - scan.ranges[follow_wall.ind2]
        if -0.04 < err < 0.04:
            vel_cmd.angular.z = 0
        else:
            vel_cmd.angular.z = err * follow_wall.gain
        vel_pub.publish(vel_cmd)
        try:
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            return None
    vel_cmd.linear.x = vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)
    machine_states["Following Wall"] = False
    machine_states["Turning Right"] = True


def take_action() -> None:
    """Decides what the robot should do based on the states of regions"""
    if not not_lost_wall:
        machine_states["Finding Wall"] = True
        machine_states["Following Wall"] = machine_states["Turning Right"] = machine_states["Turning Left"] = machine_states["Inactive"] = False
    log_state()
    if machine_states["Finding Wall"]:
        find_wall()
    elif machine_states["Following Wall"]:
        follow_wall()
    elif machine_states["Turning Right"]:
        turn_right()
    elif machine_states["Turning Left"]:
        turn_left()


def clbk_scan(msg: LaserScan) -> None:
    global scan, not_lost_wall
    scan = msg
    temp = False
    regions["front"], nlw = look_for_obstacles(0, scan)
    temp = temp or nlw
    regions["left"], nlw = look_for_obstacles(1, msg)
    temp = temp or nlw
    regions["right"], nlw = look_for_obstacles(3, msg)
    not_lost_wall = temp or nlw


def actiavation_handler(req: SetBoolRequest) -> SetBoolResponse:
    global active
    if active and req.data:
        rospy.loginfo(f"{NODE_NAME} already active!")
    elif active or req.data:
        rospy.loginfo(f"\n---- {NODE_NAME} now {'active' if req.data else 'incative'}! ----\n")
        active = req.data
        stop()
        if active:
            machine_states['Finding Wall'] = True
            machine_states['Inactive'] = machine_states['Following Wall'] = machine_states['Turning Right'] = machine_states['Turning Left'] = False
        else:
            machine_states['Inactive'] = True
            machine_states['Finding Wall'] = machine_states['Following Wall'] = machine_states['Turning Right'] = machine_states['Turning Left'] = False
    else:
        rospy.loginfo(f"\n---- {NODE_NAME} already inactive! ----\n")
    return SetBoolResponse(success=True, message="Done!")


def setup():
    rospy.init_node(NODE_NAME)
    # setting up services
    global activation_srvr
    activation_srvr = rospy.Service(ACTIVATION_SRV, SetBool, actiavation_handler)
    # setting up publishers
    global vel_pub
    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
    # setting up subscribers
    global scan_sub
    scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, clbk_scan, queue_size=1)
    # setting up durations
    global sleep
    sleep = rospy.Rate(2)
    rospy.wait_for_message(SCAN_TOPIC, LaserScan)
    # setting up follow_wall static values
    follow_wall.ind1 = int(map0to2pi(-pi / 4 - scan.angle_min) / scan.angle_increment)
    follow_wall.ind2 = int(map0to2pi(-3 * pi / 4 - scan.angle_min) / scan.angle_increment)
    follow_wall.gain = 10


def main():
    setup()

    loop_rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        if active:
            take_action()
        else:
            try:
                loop_rate.sleep()
            except rospy.ROSInterruptException:
                pass


if __name__ == "__main__":
    main()
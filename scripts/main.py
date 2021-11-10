#!/usr/bin/env python

# main driver node
import rospy
from std_srvs.srv import SetBool
from bug0.srv import UpdateGoal
from geometry_msgs.msg import Point32

# CONSTANTS
NODE_NAME = "main_driver"
# topics
SCAN_TOPIC = "scan"
ERR_TOPIC = "nav_error"
# services
FOLLOW_WALL_SRV = "fw/activate"
GO_TO_GOAL_SRV = "gtg/activate"
UPDATE_GOAL_SRV = "update_goal"

# GLOBALS
# services
follow_wall: rospy.ServiceProxy = None
go_to_goal: rospy.ServiceProxy = None
update_goal: rospy.ServiceProxy = None
# navigation
goal = Point32()

def setup():
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


def main():
    setup()


if __name__ == "__main__":
    main()

#!/usr/bin/env python

# main driver node
import rospy

# CONSTANTS
NODE_NAME = "main_driver"

def setup():
    rospy.init_node(NODE_NAME)


def main():
    setup()


if __name__ == "__main__":
    main()

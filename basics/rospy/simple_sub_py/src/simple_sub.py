#! /usr/bin/env python

import sys
import rospy
from std_msgs.msg import String

def main(args=None):
    rospy.init_node("simple_sub_node")

if __name__ == "__main__":

    try:
        main(sys.argv)
    except ROSInterruptException:
        pass

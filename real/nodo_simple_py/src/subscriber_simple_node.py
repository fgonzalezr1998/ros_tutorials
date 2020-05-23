#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32

class IntSubscriber():

    def __init__(self):
        self.sub_ = rospy.Subscriber('/publisher_simple_node/publisher1', Int32, self.callback)

    def callback(self, data):
        print(data.data)

if __name__ == "__main__":

    rospy.init_node('subscriber_simple_node')

    int_sub = IntSubscriber()
    rospy.spin()

#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32

if __name__ == "__main__":
    #Create node
    rospy.init_node('publisher_simple_node')

    rate = rospy.Rate(2) #Hz

    #crear el objeto publicador
    pub = rospy.Publisher('/publisher_simple_node/publisher1', Int32, queue_size = 1)

    msg = Int32()
    counter = 0
    while(not rospy.is_shutdown()):
        msg.data = counter
        pub.publish(msg)
        counter = counter + 1
        rate.sleep()

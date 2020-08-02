#! /usr/bin/env python

import rospy
from std_msgs.msg import String

class Publisher():
    def __init__(self):
        self.pub = rospy.Publisher("/talk", String, queue_size=1)

    def doWork(self):
        msg = String()
        msg.data = "Hello World"
        self.pub.publish(msg)

def main():
    rospy.init_node("simple_pub_node")

    publisher = Publisher()

    # 2Hz Rate:

    rate = rospy.Rate(2)
    while(not rospy.is_shutdown()):
        publisher.doWork()
        rate.sleep()

if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass

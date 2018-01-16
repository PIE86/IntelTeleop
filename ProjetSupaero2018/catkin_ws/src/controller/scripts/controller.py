#!/usr/bin/env python

import rospy
from controller.msg import *

TOPIC = 'command'

def controller():
    pub = rospy.Publisher(TOPIC, Command2D, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    dx, dy = 0.1, 0.2
    i = 0
    while not rospy.is_shutdown():
        rospy.loginfo(hello_str)
        pub.publish(dx, dy)
        rate.sleep()
        i += 1
        if i == 10:
            i = 0
            dx, dy = -dx, -dy

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

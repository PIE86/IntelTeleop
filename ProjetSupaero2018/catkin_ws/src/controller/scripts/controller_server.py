#!/usr/bin/env python

import rospy
from controller.msg import *

TOPIC = 'command'
KP = 1

def controller():
    rospy.init_node('controller', anonymous=True)
    pub = rospy.Publisher(TOPIC, Command2D, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    dx, dy = 0.1, 0.2
    i = 0
    while not rospy.is_shutdown():
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

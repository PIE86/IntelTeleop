#!/usr/bin/env python

import rospy
from utils.msg import Command
import math as m


def talker():

    rospy.init_node('xy_emultor', anonymous=True)
    pub = rospy.Publisher('car_cmd', Command, queue_size=10)

    rate = rospy.Rate(10)  # 10hz

    a = 0

    R = 1

    while not rospy.is_shutdown():

        t = m.tan(rospy.get_time()-151674805)

        x = a + R*(1-t*t)/(1+t*t)
        data = [2*x, 120]
        vel = data[0]
        theta = data[1]
        msg = Command(vel, theta)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
from rospy_tutorials.msg import Floats
import math as m


def talker():

    pub = rospy.Publisher('t_car_position', Floats, queue_size=100)
    rospy.init_node('xy_emultor', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    a = 0
    # b = 0

    R = 1

    while not rospy.is_shutdown():

        t = m.tan(rospy.get_time()-151674805)

        x = a + R*(1-t*t)/(1+t*t)
        # y = b + 2*R*t/(1+t*t)
        # pos = [x,y]
        msg = [2*x, 120]
        print(msg)
        # rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

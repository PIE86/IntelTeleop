#!/usr/bin/env python

import rospy
from rospy_tutorials.msg import Floats
import numpy as np


def talker():
    """ Emulates a topic giving a position on a clock
    """
    pub = rospy.Publisher('t_car_position_emulator', Floats, queue_size=100)
    rospy.init_node('xy_emulator', anonymous=True)
    rate = rospy.Rate(1)

    # Setup list of points to emulate
    count = 0
    number_points = 10
    points_list = [np.linspace(0, 10, number_points),
                   np.linspace(0, 5, number_points)]

    while not rospy.is_shutdown():
        # Parse points_list with counter
        count += 1
        count %= number_points
        # New point to publish on loop
        xi, yi = points_list[0][count], points_list[1][count]
        # Create message
        msg = [xi, yi]
        rospy.loginfo(msg)
        # Publish message
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

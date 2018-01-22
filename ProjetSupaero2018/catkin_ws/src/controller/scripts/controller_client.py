#!/usr/bin/env python
import rospy
from controller.msg import *

TOPIC = 'command'

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard: " + str(data.dx) + "," + str(data.dy))

def listener():

    rospy.init_node('controller_client', anonymous=True)
    rospy.Subscriber(TOPIC, Command2D, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

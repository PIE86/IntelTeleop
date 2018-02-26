#!/usr/bin/env python

"""Example script to demonstrate how to call the path_finding service"""

import rospy
from roadmap.srv import PathFinding
from geometry_msgs.msg import Point


SERVICE_NAME = 'find_path'


def ask_path(s1, s2):
    rospy.wait_for_service(SERVICE_NAME)
    # rospy.init_node('path_finding_client')
    try:
        find_path = rospy.ServiceProxy(SERVICE_NAME, PathFinding)
        resp = find_path(s1, s2)
        resp_path = resp.path
        state_path = [(p.x, p.y) for p in resp_path]
        return state_path
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == '__main__':
    s1 = Point(1., 4., 0)
    s2 = Point(6., 8., 0)
    popo = ask_path(s1, s2)
    print("Found path:")
    print(popo)

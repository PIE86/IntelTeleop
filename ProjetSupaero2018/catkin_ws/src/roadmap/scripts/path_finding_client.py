#!/usr/bin/env python

import rospy
import prm
from roadmap.srv import PathFinding, PathFindingResponse
from geometry_msgs.msg import Point


SERVICE_NAME = 'find_path'


def ask_path(s1, s2):
    rospy.wait_for_service(SERVICE_NAME)
    # rospy.init_node('path_finding_client')
    try:
        find_path = rospy.ServiceProxy(SERVICE_NAME, PathFinding)
        resp = find_path(s1, s2)
        print(resp)
        print(resp[0])
        print(resp[0].x)
        print(resp[0].y)
        return resp.vec, resp.size
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    s1 = Point(1., 4., 0)
    s2 = Point(6., 8., 0)
    ask_path(s1, s2)

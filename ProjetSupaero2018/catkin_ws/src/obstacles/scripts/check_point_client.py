#!/usr/bin/env python

import rospy
<<<<<<< HEAD:ProjetSupaero2018/catkin_ws/src/pie86_obstacles/scripts/check_point_client.py
from pie86_obstacles.srv import *
=======
import sys
from obstacles.srv import CheckPoint
>>>>>>> wjussiau:ProjetSupaero2018/catkin_ws/src/obstacles/scripts/check_point_client.py

SERVICE_NAME = 'check_point'


def check_point_client(x, y):
    rospy.wait_for_service(SERVICE_NAME)
    try:
        get_if_valid = rospy.ServiceProxy(SERVICE_NAME, CheckPoint)
        resp = get_if_valid(x, y)
        return resp.is_valid
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    else:
        print "Expecting two arguments: point coordinates x y"
        sys.exit(1)
    print "Requesting check on point: %s %s" % (x, y)
    is_valid = check_point_client(x, y)
    print "After checking point: point is valid? %s" % is_valid

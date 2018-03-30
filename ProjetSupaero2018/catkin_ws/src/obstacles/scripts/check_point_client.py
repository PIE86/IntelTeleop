#!/usr/bin/env python

import rospy
import sys
from obstacles.srv import CheckPoint


'''
Client: ask to check if point (x, y) lies in an obstacle
'''

SERVICE_NAME = 'check_point'


def check_point_client(x, y):
    rospy.wait_for_service(SERVICE_NAME)
    try:
        get_if_valid = rospy.ServiceProxy(SERVICE_NAME, CheckPoint)
        resp = get_if_valid(x, y)
        return resp.is_valid
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    else:
        print("Expecting two arguments: point coordinates x y")
        sys.exit(1)
    print("Requesting check on point: %s %s" % (x, y))
    is_valid = check_point_client(x, y)
    print("After checking point: point is valid? %s" % is_valid)

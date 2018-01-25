#!/usr/bin/env python

import rospy
import sys
from display.srv import SpawnCylinder

SERVICE_NAME = 'spawn_cylinder'


def spawn_cylinder_client(x, y, r):
    rospy.wait_for_service(SERVICE_NAME)
    try:
        spawn_cyl = rospy.ServiceProxy(SERVICE_NAME, SpawnCylinder)
        resp = spawn_cyl(x, y, r)
        return resp.success
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        r = float(sys.argv[3])
    else:
        print "Expecting 3 arguments: cylinder x y r"
        sys.exit(1)
    is_valid = spawn_cylinder_client(x, y, r)
    rospy.loginfo(str("Spawn successful: %s " % is_valid))

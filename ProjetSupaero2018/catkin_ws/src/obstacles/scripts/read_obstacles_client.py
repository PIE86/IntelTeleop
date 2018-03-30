#!/usr/bin/env python

import rospy
from obstacles.srv import ReadObstacles
import sys

'''
Client: ask to read obstacles from given file
'''


def read_obstacles_client(file_name):
    rospy.wait_for_service('read_obstacles')
    try:
        get_obstacles = rospy.ServiceProxy('read_obstacles', ReadObstacles)
        resp = get_obstacles(file_name)
        return resp.vec, resp.size
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    if len(sys.argv) == 2:
        file_name = str(sys.argv[1])
    else:
        print("%s [file_name(.obs)]" % sys.argv[0])
        sys.exit(1)
    print("Requesting obstacles from file : %s" % file_name)
    vec, size = read_obstacles_client(file_name)
    print("Read obstacles of size %s : %s" % (size, vec))

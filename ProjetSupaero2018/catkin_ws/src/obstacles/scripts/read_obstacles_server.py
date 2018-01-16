#!/usr/bin/env python

from obstacles.srv import ReadObstacles, ReadObstaclesResponse
import rospy
import obstacles_functions


def handle_obstacles(req):
    print "Returning obstacles from file [%s]" % req.file
    vec, size = obstacles_functions.read_obstacles_function(req.file)

    print "Obstacles of size %s are : [%s]" % (size, vec)
    return ReadObstaclesResponse(size, vec)


def read_obstacles_server():
    rospy.init_node('read_obstacles_server')
    rospy.Service('read_obstacles', ReadObstacles, handle_obstacles)
    print "Ready to read file.obs"
    rospy.spin()


if __name__ == "__main__":
    read_obstacles_server()

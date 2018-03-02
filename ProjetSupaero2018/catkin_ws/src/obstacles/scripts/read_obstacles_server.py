#!/usr/bin/env python

from obstacles.srv import ReadObstacles, ReadObstaclesResponse
import rospy
import obstacles_functions
import rospkg

PACKAGE_NAME = 'obstacles'

PARAM_NAME_SIZE = '/' + PACKAGE_NAME + '/obstacles_size'
PARAM_NAME_OBSTACLES = '/' + PACKAGE_NAME + '/obstacles_vec'


def handle_obstacles(req):
    print "Returning obstacles from file [%s]" % req.file
    vec, size = obstacles_functions.read_obstacles_function(req.file)

    print "Obstacles of size %s are : [%s]" % (size, vec)
    return ReadObstaclesResponse(size, vec)


def read_obstacles_server():
    rospy.init_node('read_obstacles_server')
    rospy.Service('read_obstacles', ReadObstacles, handle_obstacles)

    # Store obstacles in Parameter Server
    vec, size = obstacles_functions.read_obstacles_function(file_path)
    rospy.set_param(PARAM_NAME_OBSTACLES, vec)
    rospy.set_param(PARAM_NAME_SIZE, size)

    rospy.spin()


if __name__ == "__main__":
    ros_package = rospkg.RosPack()
    file_path = ros_package.get_path(
        PACKAGE_NAME) + '/resources/obstacles.obs'
    read_obstacles_server()

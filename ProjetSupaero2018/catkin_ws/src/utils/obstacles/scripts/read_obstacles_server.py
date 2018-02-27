#!/usr/bin/env python

from obstacles.srv import ReadObstacles, ReadObstaclesResponse
import rospy
import obstacles_functions
import rospkg

PACKAGE_NAME = 'utils'

PARAM_NAME_SIZE = '/' + PACKAGE_NAME + '/obstacles/obstacles_size'
PARAM_NAME_OBSTACLES = '/' + PACKAGE_NAME + '/obstacles/obstacles_vec'


def handle_obstacles(req):

    vec, size = obstacles_functions.read_obstacles_function(req.file)

    return ReadObstaclesResponse(size, vec)


def read_obstacles_server(file_path):
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
        PACKAGE_NAME) + '/obstacles/resources/obstacles.obs'
    read_obstacles_server(file_path)

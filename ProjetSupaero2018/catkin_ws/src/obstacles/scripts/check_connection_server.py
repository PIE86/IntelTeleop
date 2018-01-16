#!/usr/bin/env python

from obstacles.srv import CheckConnection, CheckConnectionResponse
import rospy
import obstacles_functions
import rospkg
import sys

PACKAGE_NAME = 'obstacles'

PARAM_NAME_SIZE = '/' + PACKAGE_NAME + '/obstacles_size'
PARAM_NAME_OBSTACLES = '/' + PACKAGE_NAME + '/obstacles_vec'


def check_if_valid(req):
    print "Checking if connection is valid between: (%s, %s) - (%s, %s)" \
          % (req.x1, req.y1, req.x2, req.y2)

    # Retrieve obstacles from Parameter Server
    vec = rospy.get_param(PARAM_NAME_OBSTACLES)
    size = rospy.get_param(PARAM_NAME_SIZE)

    is_valid = obstacles_functions.check_validity_connection(
        req.x1, req.y1, req.x2, req.y2, vec, size)
    print "Connection is valid: [%s]" % is_valid
    return CheckConnectionResponse(is_valid)


def check_connection_server(file_path):
    rospy.init_node('check_connection_server')
    rospy.Service('check_connection', CheckConnection, check_if_valid)

    # Store obstacles in Parameter Server
    vec, size = obstacles_functions.read_obstacles_function(file_path)
    rospy.set_param(PARAM_NAME_OBSTACLES, vec)
    rospy.set_param(PARAM_NAME_SIZE, size)

    print "Loaded obstacles from file: " + file_path
    print "Now ready to check if connections are valid"
    rospy.spin()


if __name__ == "__main__":
    if len(sys.argv) == 2:
        file_path = str(sys.argv[1])
    else:
        # Try to reach obstacle file
        ros_package = rospkg.RosPack()
        file_path = ros_package.get_path(
            PACKAGE_NAME) + '/resources/obstacles.obs'
    check_connection_server(file_path)

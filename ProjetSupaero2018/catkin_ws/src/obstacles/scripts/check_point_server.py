#!/usr/bin/env python


from obstacles.srv import CheckPoint, CheckPointResponse
import rospy
import sys
import obstacles_functions
import rospkg

PACKAGE_NAME = 'obstacles'

PARAM_NAME_SIZE = '/' + PACKAGE_NAME + '/obstacles_size'
PARAM_NAME_OBSTACLES = '/' + PACKAGE_NAME + '/obstacles_vec'


def check_if_valid(req):
    print "Checking if point is valid: [%s %s]" % (req.x, req.y)

    # Retrieve obstacles from Parameter Server
    vec = rospy.get_param(PARAM_NAME_OBSTACLES)
    size = rospy.get_param(PARAM_NAME_SIZE)

    is_valid = obstacles_functions.check_validity(req.x, req.y, vec, size)
    print "Point is valid: [%s]" % is_valid
    return CheckPointResponse(is_valid)


def check_point_server(file_path):
    rospy.init_node('check_point_server')
    rospy.Service('check_point', CheckPoint, check_if_valid)

    # Store obstacles in Parameter Server
    vec, size = obstacles_functions.read_obstacles_function(file_path)
    rospy.set_param(PARAM_NAME_OBSTACLES, vec)
    rospy.set_param(PARAM_NAME_SIZE, size)

    print "Loaded obstacles in file: " + file_path
    print "Now ready to check if point is valid"
    rospy.spin()


if __name__ == "__main__":
    if len(sys.argv) == 2:
        file_path = str(sys.argv[1])
    else:
        # Try to reach obstacle file
        ros_package = rospkg.RosPack()
        file_path = ros_package.get_path(
            PACKAGE_NAME) + '/resources/obstacles.obs'
    check_point_server(file_path)

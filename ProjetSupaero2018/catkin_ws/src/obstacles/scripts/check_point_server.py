#!/usr/bin/env python


from obstacles.srv import CheckPoint, CheckPointResponse
import rospy
import sys
import obstacles_functions
import rospkg


'''
Server: check if point (x, y) lies in an obstacle
'''

PACKAGE_NAME = 'obstacles'
PARAM_NAME_SIZE = '/' + PACKAGE_NAME + '/obstacles_size'
PARAM_NAME_OBSTACLES = '/' + PACKAGE_NAME + '/obstacles_vec'


def check_if_valid(req):
    """
    Check validity of point in req, with respect to obstacles
    :param req: (x, y)
    :return: bool is_valid
    """
    # Retrieve obstacles from ROS Parameter Server
    try:
        vec = rospy.get_param(PARAM_NAME_OBSTACLES)
        size = rospy.get_param(PARAM_NAME_SIZE)
    except KeyError:
        rospy.logerr('Obstacles parameters not set - '
                     'You may have to launch a server that reads obstacles '
                     'from an input file '
                     '(see obstacles/check_point_server)')
        return

    is_valid = obstacles_functions.check_validity(req.x, req.y, vec, size)
    return CheckPointResponse(is_valid)


def check_point_server(file_path):
    """
    Define server check_point
    :param file_path: path to get obstacles from
    """
    rospy.init_node('check_point_server')
    rospy.Service('check_point', CheckPoint, check_if_valid)

    # Store obstacles in Parameter Server
    vec, size = obstacles_functions.read_obstacles_function(file_path)
    rospy.set_param(PARAM_NAME_OBSTACLES, vec)
    rospy.set_param(PARAM_NAME_SIZE, size)

    print("Loaded obstacles in file: " + file_path)
    print("Now ready to check if point is valid")
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

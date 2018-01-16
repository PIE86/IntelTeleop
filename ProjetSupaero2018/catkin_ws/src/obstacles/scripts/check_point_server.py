#!/usr/bin/env python

<<<<<<< HEAD:ProjetSupaero2018/catkin_ws/src/pie86_obstacles/scripts/check_point_server.py
from pie86_obstacles.srv import *
=======
from obstacles.srv import CheckPoint, CheckPointResponse
>>>>>>> wjussiau:ProjetSupaero2018/catkin_ws/src/obstacles/scripts/check_point_server.py
import rospy
import sys
import obstacles_functions
import rospkg

<<<<<<< HEAD:ProjetSupaero2018/catkin_ws/src/pie86_obstacles/scripts/check_point_server.py
global PACKAGE_NAME
PACKAGE_NAME = 'pie86_obstacles'
=======
PACKAGE_NAME = 'roadmap'
>>>>>>> wjussiau:ProjetSupaero2018/catkin_ws/src/obstacles/scripts/check_point_server.py

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
<<<<<<< HEAD:ProjetSupaero2018/catkin_ws/src/pie86_obstacles/scripts/check_point_server.py
    s = rospy.Service('check_point', CheckPoint, check_if_valid)

=======
    rospy.Service('check_point', CheckPoint, check_if_valid)

    # Store obstacles in Parameter Server
>>>>>>> wjussiau:ProjetSupaero2018/catkin_ws/src/obstacles/scripts/check_point_server.py
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
<<<<<<< HEAD:ProjetSupaero2018/catkin_ws/src/pie86_obstacles/scripts/check_point_server.py
        rospackage = rospkg.RosPack()
        file_path = rospackage.get_path(PACKAGE_NAME) + '/resources/obstacles.obs'
    check_point_server(file_path)


class Obstacles:
    def __init__(self, vec=[], size=0):
        self.vec = vec
        self.size = size
=======
        # Try to reach obstacle file
        ros_package = rospkg.RosPack()
        file_path = ros_package.get_path(
            PACKAGE_NAME) + '/resources/obstacles.obs'
    check_point_server(file_path)
>>>>>>> wjussiau:ProjetSupaero2018/catkin_ws/src/obstacles/scripts/check_point_server.py

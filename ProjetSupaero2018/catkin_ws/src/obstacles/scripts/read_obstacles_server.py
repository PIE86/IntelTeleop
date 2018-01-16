#!/usr/bin/env python

<<<<<<< HEAD:ProjetSupaero2018/catkin_ws/src/pie86_obstacles/scripts/read_obstacles_server.py
from pie86_obstacles.srv import *
=======
from obstacles.srv import ReadObstacles, ReadObstaclesResponse
>>>>>>> wjussiau:ProjetSupaero2018/catkin_ws/src/obstacles/scripts/read_obstacles_server.py
import rospy
import obstacles_functions


def handle_obstacles(req):
    print "Returning obstacles from file [%s]" % req.file
    vec, size = obstacles_functions.read_obstacles_function(req.file)

<<<<<<< HEAD:ProjetSupaero2018/catkin_ws/src/pie86_obstacles/scripts/read_obstacles_server.py
    print "Obstacles of size %s are : [%s]"%(size,vec)
=======
    print "Obstacles of size %s are : [%s]" % (size, vec)
>>>>>>> wjussiau:ProjetSupaero2018/catkin_ws/src/obstacles/scripts/read_obstacles_server.py
    return ReadObstaclesResponse(size, vec)


def read_obstacles_server():
    rospy.init_node('read_obstacles_server')
    rospy.Service('read_obstacles', ReadObstacles, handle_obstacles)
    print "Ready to read file.obs"
    rospy.spin()


if __name__ == "__main__":
    read_obstacles_server()

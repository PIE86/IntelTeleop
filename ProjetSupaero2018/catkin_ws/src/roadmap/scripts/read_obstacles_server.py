#!/usr/bin/env python

from pie86_obstacles.srv import *
import rospy
import read_obstacles

def handle_obstacles(req):
    print "Returning obstacles from file [%s]"%(req.file)
    vec, size = read_obstacles.read_obstacles_function(req.file)
    
    print "Obstacles read are : [%s]"%(vec)
    return ReadObstaclesResponse(size, vec)

def read_obstacles_server():
    rospy.init_node('read_obstacles_server')
    s = rospy.Service('read_obstacles', ReadObstacles, handle_obstacles)
    print "Ready to read file.obs"
    rospy.spin()

if __name__ == "__main__":
    read_obstacles_server()
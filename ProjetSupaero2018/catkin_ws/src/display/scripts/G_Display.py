#!/usr/bin/env python

import os
import roslaunch
import rospy, tf
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32
from display.msg import Control
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *

_car_name = 'my_car'

def callbackJalon1(data):

    #rospy.loginfo('I heard %f, %f', data.data[0], data.data[1])

    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz

    msg = ModelState() 
    msg.pose.position.x = data.data[0]
    msg.pose.position.y = data.data[1]
    msg.model_name = _car_name
    pub.publish(msg)
    rate.sleep()

def callbackJalon2(data):

    #rospy.loginfo('I heard %f, %f', data.data[0], data.data[1])

    pub = rospy.Publisher('/gazebo/car_cmd', Control, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz

    vel = data.data[0]
    theta = data.data[1]

    msg = Control(vel, theta)
    pub.publish(msg)
    rate.sleep()

def listener():

    rospy.init_node('g_display', anonymous=True)

    rospy.Subscriber('t_car_position', Floats, callbackJalon2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def initWorld():

    #rospy.wait_for_service("gazebo/delete_sdf_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    #delete_sdf_model = rospy.ServicePoxy("gazebo/delete_sdf_model", DeleteModel)
    spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    with open("src/display/models/my_car/model.sdf", "r") as f:
        product_xml = f.read()

    angle = tf.transformations.quaternion_from_euler(0,0,0)
    orient = Quaternion(angle[0], angle[1], angle[2], angle[3])

    item_pose   =   Pose(Point(x = 0, y = 0, z = 0),   orient)
    spawn_sdf_model(_car_name, product_xml, "", item_pose, "world")

if __name__ == '__main__':
    
    try:
        initWorld()
        #listener()
    except rospy.ROSInterruptException:
         pass

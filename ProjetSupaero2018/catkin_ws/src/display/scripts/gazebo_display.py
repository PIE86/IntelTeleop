#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelState

MODEL_NAME = 'my_car'
CAR_POSITION_TOPIC = 't_car_position'
# CAR_POSITION_EMULATOR_TOPIC = 't_car_position_emulator'


def callback(point):
    """ Upon receiving a message, sets the state of the model
    """
    rospy.loginfo('Message through set_model_state is [%s, %s]',
                  point.x, point.y)
    # Setup publisher
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rate = rospy.Rate(10)
    # Setup message
    msg = ModelState()
    msg.pose.position.x = point.x
    msg.pose.position.y = point.y
    msg.model_name = MODEL_NAME
    # Publish message on given
    pub.publish(msg)
    rate.sleep()


def listener():
    """ Listens to topic t_car_position """
    rospy.init_node('gazebo_display', anonymous=True)
    rospy.Subscriber(CAR_POSITION_TOPIC, Point, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

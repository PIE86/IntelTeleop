#!/usr/bin/env python

import rospy
from rospy_tutorials.msg import Floats
from gazebo_msgs.msg import ModelState

MODEL_NAME = 'my_car'
CAR_POSITION_TOPIC = 't_car_position'
# CAR_POSITION_EMULATOR_TOPIC = 't_car_position_emulator'


def callback(float_msg):
    """ Upon receiving a message, sets the state of the model
    """
    rospy.loginfo('Message through set_model_state is [%s, %s]',
                  float_msg.data[0], float_msg.data[1])
    # Setup publisher
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rate = rospy.Rate(1)
    # Setup message
    msg = ModelState()
    msg.pose.position.x = float_msg.data[0]
    msg.pose.position.y = float_msg.data[1]
    msg.model_name = MODEL_NAME
    # Publish message on given
    pub.publish(msg)
    rate.sleep()


def listener():
    """ Listens to topic t_car_position """
    rospy.init_node('gazebo_display', anonymous=True)
    rospy.Subscriber(CAR_POSITION_TOPIC, Floats, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

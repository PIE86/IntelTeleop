#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from controller.msg import Command2D

COMMAND_TOPIC = 't_car_command'
CURRENT_POSITION_TOPIC = 't_car_position'


class CarModel:
    def __init__(self, init_x, init_y):
        """ Create a car given its initial position
        """
        self.x = init_x
        self.y = init_y

    def command(self, dx, dy):
        """ Change the position of the car given a control value (dx, dy)
        """
        self.x += dx
        self.y += dy


if __name__ == '__main__':
    try:
        # Create the car model
        car_model = CarModel(0.0, 0.0)

        # Callback for the command
        def callback(data):
            car_model.command(data.dx, data.dy)

        # Subscribe to the COMMAND_TOPIC
        rospy.Subscriber(COMMAND_TOPIC, Command2D, callback)

        # Create the CURRENT_POSITION_TOPIC
        publisher = rospy.Publisher(
            CURRENT_POSITION_TOPIC, Point, queue_size=10)
        rospy.init_node('car_model', anonymous=True)

        # Publish rate in Hz
        rate = rospy.Rate(10)

        # Car position message
        car_position_msg = Point()
        car_position_msg.z = 0.0

        # Publish the car position on the CURRENT_POSITION_TOPIC
        while not rospy.is_shutdown():
            car_position_msg.x = car_model.x
            car_position_msg.y = car_model.y

            rospy.loginfo('car_position: {}, {}, {}'.format(
                car_position_msg.x, car_position_msg.y, car_position_msg.z))
            publisher.publish(car_position_msg)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

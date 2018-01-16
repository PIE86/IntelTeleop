#!/usr/bin/env python
import rospy
from car_model.msg import CarPositionMsg


class CarModel:
    def __init__(self, init_x, init_y):
        """ Create a car given its initial position
        """
        self.x = init_x
        self.y = init_y

    def set_position(self, dx, dy):
        """ Change the position of the car given a control value (dx, dy)
        """
        self.x += dx
        self.y += dy


if __name__ == '__main__':
    try:
        # Create the car model
        car_model = CarModel(0.0, 0.0)

        publisher = rospy.Publisher(
            't_car_position', CarPositionMsg, queue_size=10)
        rospy.init_node('car_model', anonymous=True)

        # Publish rate in Hz
        rate = rospy.Rate(10)

        car_position_msg = CarPositionMsg()

        while not rospy.is_shutdown():
            car_position_msg.x = car_model.x
            car_position_msg.y = car_model.y

            rospy.loginfo(
                'car_position: {}, {}'.format(car_model.x, car_model.y))
            publisher.publish(car_position_msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

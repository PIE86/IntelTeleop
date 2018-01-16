#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String


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

        publisher = rospy.Publisher('t_car_position', String, queue_size=10)
        rospy.init_node('car_model', anonymous=True)

        # Publish rate in Hz
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rospy.loginfo(
                'car_position: {}, {}'.format(car_model.x, car_model.y))
            publisher.publish('{},{}'.format(car_model.x, car_model.y))
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

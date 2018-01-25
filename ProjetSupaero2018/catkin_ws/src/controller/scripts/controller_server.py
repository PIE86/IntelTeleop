#!/usr/bin/env python

import math
import rospy
from controller.msg import Command2D
from roadmap.srv import PathFinding
from geometry_msgs.msg import Point

COMMAND_TOPIC = 't_car_command'
CAR_POSI_TOPIC = 't_car_position'
PATH_FINDING_SERVICE = 'find_path'
KP = 0.1
THRES = 0.1

# CHEATING!
S1 = Point(0., 0., 0.)
S2 = Point(8., 1., 0.)


class Controller:

    def __init__(self, name):
        self.name = name
        rospy.wait_for_service(PATH_FINDING_SERVICE)
        self.state_path = []
        self.sub = rospy.Subscriber(CAR_POSI_TOPIC, Point, self.compute_cmd)
        self.pub = rospy.Publisher(COMMAND_TOPIC, Command2D, queue_size=10)

        self.state = S1
        self.goal = S2
        self.next_state_idx = 1

    def init_path(self, s1, s2):
        self.state_path = self.ask_path(s1, s2)

    def compute_cmd(self, s):
        """
        Callback of the current position topic, return a dx, dy command
        when a new state is received
        """
        self.state = s
        print('CURRENT STATE:', s)
        if len(self.state_path) > 0:
            next_state = self.state_path[self.next_state_idx]
            dist = math.sqrt((next_state.x - s.x)**2 + (next_state.x - s.x)**2)
            if dist > THRES:
                # Linear control
                dx = KP*(next_state.x - s.x)
                dy = KP*(next_state.y - s.y)
                self.pub.publish(dx, dy)
            else:
                # if next state not last state
                if self.next_state_idx < (len(self.state_path)-1):
                    self.next_state_idx += 1
                    next_state = self.state_path[self.next_state_idx]
                    print('######################')
                    print('NEW NEXT STATE!', next_state,
                          'NB', self.next_state_idx)
                    # Linear control
                    dx = KP*(next_state.x - s.x)
                    dy = KP*(next_state.y - s.y)
                    print('PUBLISH COMMAND:', dx, dy)
                    self.pub.publish(dx, dy)
                else:
                    print('######################')
                    print('Last state reached')

    def ask_path(self, s1, s2):
        try:
            # try as an attribute
            find_path = rospy.ServiceProxy(PATH_FINDING_SERVICE, PathFinding)
            resp = find_path(s1, s2)
            return resp.path
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


def euclid_points(p1, p2):
    """Take 2 geometry_msg.Point arguments and return euclidian distance"""
    return math.sqrt((p2.x-p1.x)**2 + (p2.y-p1.y)**2)


if __name__ == '__main__':
    try:
        rospy.init_node('controller', anonymous=True)
        cont = Controller(rospy.get_name())
        cont.init_path(S1, S2)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

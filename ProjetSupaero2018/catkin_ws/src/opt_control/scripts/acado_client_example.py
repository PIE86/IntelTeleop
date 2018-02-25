#!/usr/bin/env python
import rospy
from opt_control.srv import OptControl
from geometry_msgs.msg import Point

rospy.init_node('acado_client')
rospy.wait_for_service('solve_rocket')
rospy.loginfo('End of wait for rocket')

states = [
    Point(0.00e+00, 0.00e+00, 1.00e+00),
    Point(2.99e-01, 7.90e-01, 9.90e-01),
    Point(1.13e+00, 1.42e+00, 9.81e-01),
    Point(2.33e+00, 1.69e+00, 9.75e-01),
    Point(3.60e+00, 1.70e+00, 9.73e-01),
    Point(4.86e+00, 1.70e+00, 9.70e-01),
    Point(6.13e+00, 1.70e+00, 9.68e-01),
    Point(7.39e+00, 1.70e+00, 9.65e-01),
    Point(8.66e+00, 1.70e+00, 9.63e-01),
    Point(9.67e+00, 8.98e-01, 9.58e-01),
    Point(1.00e+01, 0.00e+00, 9.49e-01),
]
controls = [
    Point(1.10e+00, 0., 0.),
    Point(1.10e+00, 0., 0.),
    Point(1.10e+00, 0., 0.),
    Point(5.78e-01, 0., 0.),
    Point(5.78e-01, 0., 0.),
    Point(5.78e-01, 0., 0.),
    Point(5.78e-01, 0., 0.),
    Point(5.78e-01, 0., 0.),
    Point(2.12e-01, 0., 0.),
    Point(1.10e+00, 0., 0.),
    Point(1.10e+00, 0., 0.),
]
cost = 7.44e+00
try:
    opt_control = rospy.ServiceProxy('solve_rocket', OptControl)
    p1 = Point(0, 0, 1)
    p2 = Point(10, 0, 0)
    # resp = opt_control(p1, p2, [], [], 0)
    resp = opt_control(p1, p2, states, controls, cost)

    print(resp.success)
    print('Path length:', len(resp.states))
    print(resp.states)
    print(resp.controls)
except rospy.ServiceException, e:
    print "Service call failed: %s" % e

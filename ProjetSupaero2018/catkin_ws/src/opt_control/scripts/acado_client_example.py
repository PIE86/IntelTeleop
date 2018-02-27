#!/usr/bin/env python
import rospy
from opt_control.srv import OptControl
from geometry_msgs.msg import Point

from test_trajectories import (states, controls, cost,
                               states_shortest, controls_shortest, cost_shortest,
                               states_shortest_without_init, controls_shortest_without_init, cost_shortest_without_init)

rospy.init_node('acado_client')
rospy.wait_for_service('solve_rocket')
rospy.loginfo('End of wait for rocket')

states = [Point(*x) for x in states]
controls = [Point(*x) for x in controls]
states_shortest = [Point(*x) for x in states_shortest]
controls_shortest = [Point(*x) for x in controls_shortest]
states_shortest_without_init = [Point(*x) for x in states_shortest_without_init]
controls_shortest_without_init = [Point(*x) for x in controls_shortest_without_init]


try:
    opt_control = rospy.ServiceProxy('solve_rocket', OptControl)

    # Failure rocket example
    # p1 = Point(6.394000e+00, -5.500000e-02, 2.750000e-01)
    # p2 = Point(2.279000e+00, 4.210000e-01, 8.000000e-02)

    # success rocket example, corresonds to states and controls init vectors
    p1 = Point(0, 0, 1)
    p2 = Point(10, 0, 0)
    # p1 = Point(1, 0, 1)
    # p2 = Point(2, 1, 0)
    # p1 = Point(.922, 0.435, 2.651)
    # p2 = Point(8.764, 1.573, 4.118)
    # with init
    # resp = opt_control(p1, p2, states, controls, cost)

    # Shortest path example
    # p1 = Point(2.232, 1.226, 0.677)
    # p2 = Point(9.572, 0.506, 0.26690096)
    # resp = opt_control(p1, p2, states_shortest, controls_shortest, cost_shortest)
    # resp = opt_control(p1, p2, states_shortest_without_init,
    #                    controls_shortest_without_init,
    #                    cost_shortest_without_init)

    # without init
    resp = opt_control(p1, p2, [], [], 0)

    print(resp.success)
    print('Path length:', len(resp.states))
    print(resp.states)
    print(resp.controls)
    print(resp.time)
except rospy.ServiceException as e:
    print("Service call failed: %s" % e)

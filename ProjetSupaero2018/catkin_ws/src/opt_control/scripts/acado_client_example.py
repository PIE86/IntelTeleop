#!/usr/bin/env python
import numpy as np
import rospy
from opt_control.srv import OptControl

# from test_trajectories import (
#         states, controls, cost,
#         states_shortest, controls_shortest, cost_shortest,
#         states_shortest_without_init, controls_shortest_without_init,
#         cost_shortest_without_init)

NX = 4
NU = 2

rospy.init_node('ocp_solve_client')
rospy.wait_for_service('solve_ocp')
rospy.loginfo('End of wait for ocp')

# states = np.array(states)
# controls = np.array(controls)
# states_shortest = np.array(states_shortest)
# controls_shortest = np.array(controls_shortest)
# states_shortest_without_init = np.array(states_shortest_without_init)
# controls_shortest_without_init = np.array(controls_shortest_without_init)

try:
    opt_control = rospy.ServiceProxy('solve_ocp', OptControl)

    # Failure rocket example
    # p1 = np.array((6.394000e+00, -5.500000e-02, 2.750000e-01))
    # p2 = np.array((2.279000e+00, 4.210000e-01, 8.000000e-02))

    # success rocket example, corresponds to states and controls init vectors
    # p1 = np.array((0, 0, 1))
    # p2 = np.array((10, 0, 0))
    # p1 = np.array((1, 0, 1))
    # p2 = np.array((2, 1, 0))
    p1 = np.array((0, 0, 0, 1.2))
    p2 = np.array((10, 10, 1, 1.4))
    # p1 = np.array((.922, 0.435, 2.651))
    # p2 = np.array((8.764, 1.573, 4.118))
    # with init
    #  resp = opt_control(p1, p2, states,
    # Shortest path example
    # p1 = np.array((2.232, 1.226, 0.677))
    # p2 = np.array((9.572, 0.506, 0.26690096))
    # resp = opt_control(p1, p2, states_shortest, controls_shortest,
    #                    cost_shortest, NX, NU)
    # resp = opt_control(p1, p2, states_shortest_without_init,
    #                    controls_shortest_without_init,
    #                    cost_shortest_without_init, NX, NU)

    # without init
    resp = opt_control(p1, p2, [], [], 0, NX, NU)

    print(resp.success)
    print('Path length:', len(resp.states))
    print(resp.states)
    print(resp.controls)
    print(resp.time)
except rospy.ServiceException as e:
    print("Service call failed: %s" % e)

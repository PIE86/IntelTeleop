#!/usr/bin/env python

"""
Example script explaining how to create a service for the opt_control service.

To be launched with:
roslaunch roadmap test_opt_control.launch --screen
"""

import numpy as np
import rospy
import actionlib
import matplotlib.pyplot as plt

from networks import Networks
from irepa import NX, NU, X_MIN, X_MAX, U_MIN, U_MAX

from roadmap.msg import OptControlAction, OptControlGoal
plt.rc('legend', **{'fontsize': 10})


OPT_CONTROL_SERVER = 'solve_ocp'

rospy.init_node('ocp_solve_client')

ocp_client = actionlib.SimpleActionClient(OPT_CONTROL_SERVER,
                                          OptControlAction)
ocp_client.wait_for_server()
rospy.loginfo('End of wait for rocket')


print('Creating NN')
estimator = Networks(NX, NU,
                     x_range=np.array([X_MIN, X_MAX]),
                     u_range=np.array([U_MIN, U_MAX]))
estimator.load()
print('Weights loaded')

Xs = []
Vs = []


def callback(state, resp):
    global Xs
    global Vs
    X = np.array(resp.states).reshape(len(resp.states)//NX, NX)
    U = np.array(resp.controls).reshape(len(resp.controls)//NU, NU)
    print()
    print()
    print(resp.success)
    print('Path length:', len(resp.states)//NX)
    print(X)
    print(U)
    print(resp.time)
    Vs.append(resp.time)
    Xs.append(X)


# SIMPLE AND GOOD
# start = np.array((2, 2, 0))
# end = np.array((12, 5, 0))

# VERY DIFFERENT PATHS -> VERY GOOD
# start = np.array((7, 1, -1))
# end = np.array((14, 14, 1.5))

# BUG: constraints not respected
# start = np.array((10, 8, 3.14))
# end = np.array((10, 15, 2))

# INTERESTING
# start = np.array((10, 8, 3.14))
# end = np.array((10, 15, 0))

# VERY LONG -> OK
# start = np.array((2, 2, 0))
# end = np.array((17, 17, 3))

# VERY LONG 2 -> OK
# start = np.array((2, 2, 0))
# end = np.array((13, 18, 3))

# VERY LONG 2 -> OK
# start = np.array((1, 1, 0))
# end = np.array((17, 19, 3))

# HARD -> No init does not work
# BUG: strange, plots function but no traj retrieved
# start = np.array((1, 19, -1.5))
# end = np.array((12, 10, -3))


# obstacle positive angle -> fail without init
# start = np.array([2, 2, np.pi/2])
# end = np.array([11, 15, 0])

# NOPE
start = np.array([2, 2, np.pi/2])
end = np.array([10, 17, 0])

print('Start end')
print(start)
print(end)

print('Without INIT')
Xe, Ue, Ve = [], [], 0
goal = OptControlGoal(
    list(start),
    list(end),
    Xe, Ue, Ve, NX, NU)

ocp_client.send_goal(goal, callback)
ocp_client.wait_for_result(rospy.Duration.from_sec(3.0))


print('With INIT')
Xe, Ue, Ve = estimator.trajectories(start, end)
print()
print('Network')
print()

Xe_f = Xe.flatten()
Ue_f = Ue.flatten()
goal = OptControlGoal(
    list(start),
    list(end),
    Xe_f, Ue_f, Ve, NX, NU)

ocp_client.send_goal(goal, callback)
ocp_client.wait_for_result(rospy.Duration.from_sec(3.0))

plt.xlim(0, 20)
plt.ylim(0, 20)

ax = plt.gca()
# Start
c = plt.Circle((start[0], start[1]), radius=0.3, color="red")
ax.add_artist(c)
c = plt.Circle((end[0], end[1]), radius=0.3, color="green")
ax.add_artist(c)

# Obstacles
c = plt.Circle((4, 16), radius=2, color="blue")
ax.add_artist(c)
c = plt.Circle((10, 12), radius=2, color="blue")
ax.add_artist(c)


plt.plot(Xe[:, 0], Xe[:, 1],
         label='Estimator, T: '+str(round(Ve[0, 0], 2))+'s', color='violet')
plt.plot(Xs[0][:, 0], Xs[0][:, 1],
         label='No init, T: '+str(round(Vs[0], 2))+' s', color='orange')
plt.plot(Xs[1][:, 0], Xs[1][:, 1],
         label='Init, T: '+str(round(Vs[1], 2))+' s', color='lightblue')
plt.legend()

plt.grid(True)
plt.show()

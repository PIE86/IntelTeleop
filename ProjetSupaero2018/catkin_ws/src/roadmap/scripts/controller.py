#!/usr/bin/env python

import numpy as np
import rospy
import actionlib
from opt_control.msg import Control, OptControlAction, OptControlGoal

from networks import Networks, resample
from irepa import NX, NU, X_MIN, X_MAX, U_MIN, U_MAX

OPT_CONTROL_ACTION_SERVER = 'solve_ocp'
#
# rospy.wait_for_service(OPT_CONTROL_SERVICE)
# rospy.loginfo('End of wait for ocp')

# Control frequency
CPS = 1


class Controller:

    def __init__(self):
        self.estimator = Networks(NX, NU,
                                  x_range=np.array([X_MIN, X_MAX]),
                                  u_range=np.array([U_MIN, U_MAX]))
        self.estimator.load()
        # Last state trajectory calculated
        self.X = np.array([])
        self.U = np.array([])
        self.time = 0
        self.current_state = np.zeros(NX)
        self.end = np.zeros(NX)
        # time from the start of the current trajectory in ns
        self.t = 0  # useless as attribute?
        self.t_idx = 0

        self.client = actionlib.SimpleActionClient(OPT_CONTROL_ACTION_SERVER,
                                                   OptControlAction)
        self.client.wait_for_server()
        self.pub = rospy.Publisher('controller', Control, queue_size=10)

    def next_control(self):
        """Choose next control depending on the current_state and self.U
        Callback to service controller"""
        self.t += 1 / CPS  # Maybe useless after init
        self.t_idx += 1
        u = self.U[self.t_idx, :]
        print('Control:', u)
        # self.pub(u)
        return u

    def update_trajectory(self, state, resp):
        """Callback to topic simulation"""
        print('UPDATE TRAJECTORYYYYY')
        print(state)
        print(resp)

        if resp.success:
            X = np.array(resp.states).reshape(len(resp.states)//NX, NX)
            U = np.array(resp.controls).reshape(len(resp.controls)//NU, NU)
            self.time = resp.time
            # Resample the trajectories
            nb_control = int(resp.time * CPS) + 1
            self.X = resample(X, nb_control)  # maybe not necessary
            self.U = resample(U, nb_control)
            tend = rospy.get_rostime()
            self.t = tend.secs - self.tstart.secs
            print(self.t)
            self.t_idx = int(self.t * CPS)

        else:
            # TODO
            print()
            print('FAILURE OF CONTROL!!!!')
            print()

    def call_update_trajectory_action(self):
        """
        Call ACADO warm started by the estimator.
        Update the current trajectory (X, U, time).
        Maybe start a timer at the beginning?
        """
        self.tstart = rospy.get_rostime()
        Xe, Ue, Ve = self.estimator.trajectories(self.current_state, self.end)

        Xe = Xe.flatten()
        Ue = Ue.flatten()
        # print(Xe)
        # print(Ue)
        # print(Ve)

        goal = OptControlGoal(
            list(self.current_state),
            list(self.end),
            Xe, Ue, Ve, NX, NU)
        # Fill in the goal here
        self.client.send_goal(goal, self.update_trajectory)
        # self.client.wait_for_result(rospy.Duration.from_sec(5.0))  # Nope!

    def new_end(self, end_state):
        """More after that?"""
        self.end = end_state
        self.call_update_trajectory_action()


if __name__ == '__main__':
    rospy.init_node('tototo', anonymous=True)
    rate = rospy.Rate(CPS)  # 10hz
    print('CREATE CONTROLLER')
    controller = Controller()

    # TODO: start not
    print('FIX CURRENT STATE AND END')
    controller.current_state = np.array((1, 0, 1))
    controller.end = np.array((2, 1, 0))

    while not rospy.is_shutdown():
        controller.call_update_trajectory_action()
        rate.sleep()
        controller.next_control

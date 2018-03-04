#!/usr/bin/env python

import numpy as np
import rospy
import actionlib
from roadmap.msg import OptControlAction, OptControlGoal
from display.msg import Command, State

from networks import Networks, resample
from irepa import NX, NU, X_MIN, X_MAX, U_MIN, U_MAX

OPT_CONTROL_SERVER = 'solve_ocp'
COMMAND_TOPIC = '/car_control/command'
STATE_TOPIC = '/car_control/state'

# rospy.wait_for_service(OPT_CONTROL_SERVICE)
# rospy.loginfo('End of wait for ocp')

# Control frequency
CPS = 4
# 4 Hz -> UPDATE every 40 iterations -> ~ 0.4 Hz = 2.5s
UPDATE_TIMES = 10*CPS


class Controller:

    def __init__(self):
        self.estimator = Networks(NX, NU,
                                  x_range=np.array([X_MIN, X_MAX]),
                                  u_range=np.array([U_MIN, U_MAX]))
        self.estimator.load()
        # Last state trajectory calculated
        self.X = np.array([])
        self.U = np.array([])
        self.u = np.zeros(NU)
        self.time = 0
        self.current_state = np.zeros(NX)
        self.end = np.zeros(NX)
        # time from the start of the current trajectory in ns
        self.t = 0  # useless as attribute?
        self.t_idx = 0

        self.ocp_client = actionlib.SimpleActionClient(OPT_CONTROL_SERVER,
                                                       OptControlAction)
        self.ocp_client.wait_for_server()
        self.pub = rospy.Publisher(COMMAND_TOPIC, Command, queue_size=10)
        rospy.Subscriber(STATE_TOPIC, State, self.update_state)

        # test state rate
        self.x_rate_test = 0
        self.t1 = rospy.get_rostime()
        self.t2 = rospy.get_rostime()

    def next_control(self):
        """Choose next control depending on the current_state and self.U
        Callback to service controller"""
        self.t += 1 / CPS  # Maybe useless after init
        self.t_idx += 1
        if self.t_idx < self.U.shape[0]:
            self.u = self.U[self.t_idx, :]
        print('  CONTROL:', self.u)
        self.pub.publish(self.u)
        return self.u

    def update_state(self, msg):
        # try:
        #     print('STATE received:', msg, 'End traj:', self.X[-1])
        # except Exception:
        #     pass
        self.current_state = np.array(msg.x)

    def update_trajectory(self, state, resp):
        """Callback to topic simulation"""
        # print('UPDATE TRAJECTORYYYYY')
        # print(state)
        # print(resp)

        if resp.success:
            X = np.array(resp.states).reshape(len(resp.states)//NX, NX)
            U = np.array(resp.controls).reshape(len(resp.controls)//NU, NU)
            self.time = resp.time
            # Resample the trajectories
            nb_control = int(resp.time * CPS) + 1
            print('X ACADO', X.shape)
            print(X)
            self.X = resample(X, nb_control)  # maybe not necessary
            print('X resampled', self.X.shape)
            print(self.X)
            self.U = resample(U, nb_control)
            tend = rospy.get_rostime()
            self.t = (tend.nsecs - self.tstart.nsecs)/1e9
            print('UPDATE TOOK', self.t, 'secs')
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
        print()
        print('current_state, end')
        print(self.current_state, self.end)
        print('Xe')
        print(Xe)
        # print(Ue)
        # print(Ve)

        goal = OptControlGoal(
            list(self.current_state),
            list(self.end),
            Xe, Ue, Ve, NX, NU)
        # Fill in the goal here
        self.ocp_client.send_goal(goal, self.update_trajectory)
        # Nope!
        # self.ocp_client.wait_for_result(rospy.Duration.from_sec(5.0))

    # def new_end(self, end_state):
    #     """More after that?"""
    #     self.end = end_state
    #     self.call_update_trajectory_action()


if __name__ == '__main__':
    rospy.init_node('tototo', anonymous=True)
    rate = rospy.Rate(CPS)  # 10hz
    print('CREATE CONTROLLER')
    controller = Controller()

    # TODO: start not
    print('FIX CURRENT STATE AND END')
    start = np.array((4, 6, 0))
    controller.current_state = start
    end1 = np.array((12, 8, 0))
    controller.end = end1

    i = 0

    while not rospy.is_shutdown():
        i += 1
        # print('EVERY')
        if i % UPDATE_TIMES == 0:
            # print('ONLY')
            controller.call_update_trajectory_action()
            i = 0
        controller.next_control()
        rate.sleep()

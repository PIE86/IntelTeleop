#!/usr/bin/env python

"""
Online controller to test the trajectory estimator built using IREPA.
Work locked with the simulation to produce control according to the current
state and the end state. At each iteration a state is received from the
simulation and a control is sent. Optimal trajectories are generated at a
lower frequency to prevent optimal control node overloading. This call is
asynchronious, meaning that the control trajectory is computed in a non
blocking way to enable the controls to be sent continuously.

To be launched with:
roslaunch demo_launch jalon2_online.launch --screen

Then wait for Gazebo to display the the environment. When it is ready press any
key to start the visualization.

Start and end states are hard coded both here and in
display/scripts/init_world.py
"""

import time
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

# Control frequency (Hz)
CPS = 10


class Controller:

    """
    Online controller sending commands/controls to the simulation node
    """

    def __init__(self):
        """
        Load the irepa built estimator then create clients for simulation
        and optimal control nodes
        """
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
        self.t_idx = 0
        # update trajectory every update_times iteration
        self.update_times = 10

        self.ocp_client = actionlib.SimpleActionClient(OPT_CONTROL_SERVER,
                                                       OptControlAction)
        self.ocp_client.wait_for_server()
        self.pub = rospy.Publisher(COMMAND_TOPIC, Command, queue_size=10)
        rospy.Subscriber(STATE_TOPIC, State, self.update_state)

        # test state rate
        self.t1 = rospy.get_rostime()
        self.t2 = rospy.get_rostime()

    def next_control(self):
        """
        Choose next control to send to the simulation.
        """
        self.t_idx += 1
        if self.t_idx < self.U.shape[0]:
            self.u = self.U[self.t_idx, :]
            print('  CONTROL:', self.u)
        else:
            print('  !! No more control --> previous')
        self.pub.publish(self.u)
        return self.u

    def update_state(self, msg):
        """
        Callback function for the simulation Subscriber.
        """
        # print('STATE received:', msg, 'End traj:', self.X[-1])
        self.current_state = np.array(msg.x)

    def update_trajectory(self, state, resp):
        """
        Callback function for the optimal control action server.

        Once a control trajectory is received, it is resampled using trajectory
        time so that the time difference between to consecutive controls is
        at the CPS.

        :param state: state of the action server (nothing to do with
                      the state of the system)
        :param resp: response of the action server containing
                     - states: states trajectory
                     - controls: controls trajectory
                     - time: time length of the trajectory
        """
        # print('UPDATE TRAJECTORYYYYY')
        # print(state)
        # print(resp)

        if resp.success:
            X = np.array(resp.states).reshape(len(resp.states)//NX, NX)
            U = np.array(resp.controls).reshape(len(resp.controls)//NU, NU)
            self.time = resp.time

            dt_acado = self.time/(X.shape[0]-1)
            # Resample the trajectories
            nb_control = int(resp.time * CPS) + 1
            print('X ACADO', X.shape)
            # print(X)
            self.X = resample(X, nb_control)  # maybe not necessary
            print('X resampled', self.X.shape)
            # print(self.X)
            self.U = resample(U, nb_control)
            tend = time.time()
            t_calc = (tend - self.tstart)
            self.t_idx = int(t_calc * CPS)

            print()
            print()
            print('IMPORTANT')
            print('Size traj', X.shape[0])
            print('Time traj', resp.time)
            print('Dt acado', dt_acado)
            print('nb_control', nb_control)
            print('UPDATE TOOK', t_calc, 'secs')
            print(self.t_idx)

        else:
            # TODO
            print()
            print('FAILURE OF CONTROL!!!!')
            print()

    def call_update_trajectory_action(self):
        """
        Call ACADO warm started by the estimator. The result is handled by
        update_trajectory function.
        """
        self.tstart = time.time()
        # self.tstart = rospy.get_rostime()
        Xe, Ue, Ve = self.estimator.trajectories(self.current_state, self.end)

        Xe = Xe.flatten()
        Ue = Ue.flatten()
        print()
        print('current_state, end:', self.current_state, self.end)
        print()
        # print('Xe')
        # print(Xe)
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

    def start_control(self):
        """
        Start the control loop at rate CPS
        """
        i = 0
        t1 = time.time()
        while not rospy.is_shutdown():
            i += 1
            # print('EVERY')
            if i % self.update_times == 0:
                t2 = time.time()
                print('Time since last update:', t2-t1)
                t1 = t2
                # print('ONLY')
                self.call_update_trajectory_action()
                i = 0
            self.next_control()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('controller_node', anonymous=True)
    rate = rospy.Rate(CPS)
    print('CREATE CONTROLLER')
    controller = Controller()

    # TODO: start not
    print('FIX CURRENT STATE AND END')
    start = np.array((2, 2, 0))
    controller.current_state = start
    end = np.array((12, 4, 0))
    controller.end = end

    input('\nPress key when gazebo is ready\n')

    controller.start_control()

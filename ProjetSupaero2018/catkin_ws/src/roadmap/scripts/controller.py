#!/usr/bin/env python

"""
Online controller to test the trajectory estimator built using IREPA.
Work locked with the simulation to produce control according to the current
state and the end state. At each iteration a state is received from the
simulation and a control is sent. More sparsely, a end state is also received
so that the implementation can take into account an objective change
Optimal trajectories are generated at a lower frequency to prevent optimal
control node overloading. This call is asynchronious, meaning that the control
trajectory is computed in a non blocking way to enable the controls to be sent
continuously.
Upon reception, the trajectory is resampled with a linear interpolation so that
the list of commands corresponds to the control frequency. The first control
to be sent is then chosen according to the calculation time and the trajectory
time (e.g. if calculation time is equal to 0, the first is chosen). The SHIFT
enable/disable this behaviour for testing purposes.

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
from irepa import NX, NU, X_MIN, X_MAX, U_MIN, U_MAX, euclid

OPT_CONTROL_SERVER = 'solve_ocp'
COMMAND_TOPIC = '/car_control/command'
CURRENT_STATE_TOPIC = '/car_control/state'
END_STATE_TOPIC = 'end_state'

# Control Per Second (Hz) == State Frequency
CPS = 20
# Trajectory Update Per Second (Hz)
TUPS = 3
STOP_RADIUS = 0.4
UPDATE_TIME_THRES = 0.8

# True to use warm-start the solver with the estimator
ESTIMATOR_INIT = False
# For test purposes: if true, the first control to be sent is calculated
# else the first is sent
SHIFT = True


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
        self.end_state = np.zeros(NX)
        # time from the start of the current trajectory in ns
        self.t_idx = 0
        # update trajectory every update_times iteration
        self.update_times = int(CPS / TUPS)

        self.ocp_client = actionlib.SimpleActionClient(OPT_CONTROL_SERVER,
                                                       OptControlAction)
        self.ocp_client.wait_for_server()
        self.pub = rospy.Publisher(COMMAND_TOPIC, Command, queue_size=10)
        rospy.Subscriber(CURRENT_STATE_TOPIC, State, self.update_current_state)
        rospy.Subscriber(END_STATE_TOPIC, State, self.update_end_state)

        # test state rate
        self.t1 = rospy.get_rostime()
        self.t2 = rospy.get_rostime()

        # control steps
        self.stop_update = False
        self.stop_controls = False
        self.started = False

    def next_control(self):
        """
        Choose next control to send to the simulation.
        """
        self.t_idx += 1
        if euclid(self.current_state, self.end_state) < STOP_RADIUS:
            self.stop_update = True
            self.stop_controls = True
        else:
            self.stop_update = False
            self.stop_controls = False

        if self.stop_controls:
            self.u = np.zeros(NU)
        else:
            if self.t_idx < self.U.shape[0]:
                self.u = self.U[self.t_idx, :]
                # print('  CONTROL:', self.u)
            else:
                if self.started:
                    print('  !! No more control --> previous')
        self.pub.publish(self.u)
        return self.u

    def update_current_state(self, msg):
        """
        Callback function for the simulation Subscriber for current state.
        """
        # print('CURRENT STATE received:', msg.x)
        self.current_state = np.array(msg.x)

    def update_end_state(self, msg):
        """
        Callback function for the simulation Subscriber for end state.
        """
        # print('END STATE received:', msg.x)
        self.end_state = np.array(msg.x)

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
        self.started = True  # control started at the first result
        if resp.success:
            X = np.array(resp.states).reshape(len(resp.states)//NX, NX)
            U = np.array(resp.controls).reshape(len(resp.controls)//NU, NU)
            self.time = resp.time
            # if resp.time < UPDATE_TIME_THRES:
            #     self.stop_update = True

            dt_acado = self.time/(X.shape[0]-1)
            nb_control = int(resp.time * CPS) + 1
            self.X = resample(X, nb_control)
            self.U = resample(U, nb_control)
            tend = time.time()
            t_calc = (tend - self.tstart)
            self.t_idx = int(t_calc * CPS) if SHIFT else 0

            print()
            print('RESULT TRAJECTORY')
            print('UPDATE TOOK:', round(t_calc, 2))
            print('TIME TRAJ:  ', round(resp.time, 2))
            print('Dt acado', dt_acado, 'nb_control', nb_control,
                  'SIZE X', X.shape[0])
            print(self.t_idx)

        else:
            print()
            print('FAILURE OF CONTROL!!!!')
            print()

    def call_update_trajectory_action(self):
        """
        Call ACADO warm started by the estimator. The result is handled by
        update_trajectory function.
        """
        self.tstart = time.time()
        if ESTIMATOR_INIT:
            Xe, Ue, Ve = self.estimator.trajectories(self.current_state,
                                                     self.end_state)
            Xe = Xe.flatten()
            Ue = Ue.flatten()
        else:
            Xe, Ue, Ve = [], [], 0

        goal = OptControlGoal(
            list(self.current_state),
            list(self.end_state),
            Xe, Ue, Ve, NX, NU)
        self.ocp_client.send_goal(goal, self.update_trajectory)

    def start_control(self):
        """
        Start the control loop at rate CPS
        """
        print('Control started')
        rate = rospy.Rate(CPS)
        i = 0
        while not rospy.is_shutdown():
            i += 1
            if i % self.update_times == 0:
                if not self.stop_update:
                    self.call_update_trajectory_action()
                i = 0
            self.next_control()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('controller_node', anonymous=True)
    print('CREATE CONTROLLER')
    controller = Controller()

    input('\nPress ENTER when gazebo is ready\n')

    controller.start_control()

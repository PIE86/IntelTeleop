#!/usr/bin/env python

"""
Script ploting 3 graphs exploring the relation between calculation time
for the opt control, actual time of the trajectories and euclidian distance
between start and end states. The opt_control can be initialized by the
estimator so training should have been occured beforehand.

To be launched with:
roslaunch roadmap test_latency_acado_init.launch --screen
"""

import sys
import time
import numpy as np
from matplotlib import pyplot as plt
import rospy
from roadmap.msg import OptControlGoal

from irepa import Irepa, NX, NU

OPT_CONTROL_SERVER = 'solve_ocp'

NB_SAMPLES = 400
# Initialize with the estimator
INITIALIZE = True


class LatencyTest:

    def __init__(self):
        rospy.init_node('test_latency', anonymous=True)

        rospy.wait_for_service('create_samples')
        # sampling_client = rospy.ServiceProxy('create_samples', Samples)
        # self.ocp_client = actionlib.SimpleActionClient(OPT_CONTROL_SERVER,
        #                                                OptControlAction)
        # self.ocp_client.wait_for_server()
        # rospy.loginfo('End of wait for ocp action server')

        self.irepa = Irepa()
        self.irepa.estimator.load()

        self.calc_times = []
        self.traj_times = []
        self.euclid = []

    def test_and_plot_results(self, end, nb_samples):
        """Connect the samples and plot the results."""
        self.test(end, nb_samples)
        self.plot_results()

    def test(self, end, nb_samples):
        """
        Generate nb_samples random states in the state space and tries to
        them to the same end state. Then generate calcultation and trajectory
        times and euclidian distance.
        """

        samples = self.irepa.sample(nb_samples)

        print('Start in 2 seconds')
        time.sleep(2)
        for i, state in enumerate(samples):
            sys.stdout.write("\r{}%".format(
                             round(100*float(i)/len(samples), 0)))
            sys.stdout.flush()
            if INITIALIZE:
                Xe, Ue, Ve = self.irepa.estimator.trajectories(state, end)
                Xe = Xe.flatten()
                Ue = Ue.flatten()
            else:
                Xe, Ue, Ve = [], [], 0

            goal = OptControlGoal(
                list(state),
                list(end),
                Xe, Ue, Ve, NX, NU)
            t1 = time.time()
            self.irepa.ocp_client.send_goal(goal, self.callback)
            self.irepa.ocp_client.wait_for_result(rospy.Duration.from_sec(5.0))
            t2 = time.time()
            self.calc_times.append(round(t2-t1, 2))
            self.euclid.append(round(self.irepa.euclid(state, end), 2))

    def plot_results(self):
        euclid_arr = np.array(self.euclid)
        calc_times_arr = np.array(self.calc_times)
        times_arr = np.array(self.traj_times)

        # 1rst plot
        plt.plot(euclid_arr, calc_times_arr,
                 marker='.', linestyle='', label='Calculation times')
        plt.xlabel('euclidian dist')
        plt.ylabel('time (s)')
        plt.title("""Calculation time = f(euclidian distance)
                  {} samples""".format(NB_SAMPLES))
        plt.xlim(0, 20)
        plt.ylim(0, 2)
        plt.legend()
        plt.show()

        # 2nd plot
        plt.plot(euclid_arr, times_arr,
                 marker='.', linestyle='', label='Trajectory times')
        plt.xlabel('euclidian dist')
        plt.plot(euclid_arr, times_arr - calc_times_arr,
                 marker='.', linestyle='', label='Traj - Calc times')
        plt.axhline(y=0, color='black', linestyle='-')
        plt.xlabel('euclidian dist')
        plt.ylabel('time (s)')
        plt.title("""Trajectory time = f(euclidian distance)
                  {} samples""".format(NB_SAMPLES))
        plt.xlim(0, 20)
        plt.ylim(-2, 20)
        plt.legend()
        plt.show()

        # 3rd plot
        plt.plot(times_arr,  calc_times_arr,
                 marker='.', linestyle='')
        plt.xlabel('Trajectory time (s)')
        plt.ylabel('Calculation time (s)')
        plt.title("""Calculation time = f(Trajectory time)
                  {} samples""".format(NB_SAMPLES))
        plt.legend()
        plt.xlim(0, 20)
        plt.ylim(0, 2)
        plt.show()

    def callback(self, status, resp):
        self.traj_times.append(round(resp.time, 2))


if __name__ == '__main__':
    end = np.array((12, 4, 0))
    ltest = LatencyTest()
    ltest.test_and_plot_results(end, NB_SAMPLES)

    rospy.spin()

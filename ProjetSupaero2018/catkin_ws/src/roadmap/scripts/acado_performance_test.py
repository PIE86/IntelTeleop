#!/usr/bin/env python

import time
import numpy as np
from matplotlib import pyplot as plt
import rospy
import actionlib
from roadmap.msg import OptControlAction, OptControlGoal
from opt_control.srv import Samples

from irepa import Irepa, NX, NU

OPT_CONTROL_SERVER = 'solve_ocp'

NB_SAMPLES = 200


class LatencyTest:

    def __init__(self):
        rospy.init_node('test_latency', anonymous=True)

        rospy.wait_for_service('create_samples')
        sampling_client = rospy.ServiceProxy('create_samples', Samples)
        self.ocp_client = actionlib.SimpleActionClient(OPT_CONTROL_SERVER,
                                                       OptControlAction)
        self.ocp_client.wait_for_server()
        rospy.loginfo('End of wait for ocp action server')

        self.irepa = Irepa(self.ocp_client, sampling_client)
        self.irepa.estimator.load()

        self.calc_times = []
        self.times = []
        self.euclid_dist = []

    def test(self, end):
        samples = self.irepa.sample(NB_SAMPLES)

        print('Start in 2 seconds')
        time.sleep(2)
        for state in samples:
            Xe, Ue, Ve = self.irepa.estimator.trajectories(state, end)

            Xe = Xe.flatten()
            Ue = Ue.flatten()

            goal = OptControlGoal(
                list(state),
                list(end),
                Xe, Ue, Ve, NX, NU)
            t1 = time.time()
            self.ocp_client.send_goal(goal, self.callback)
            self.ocp_client.wait_for_result(rospy.Duration.from_sec(5.0))
            t2 = time.time()
            self.calc_times.append(round(t2-t1, 2))
            self.euclid_dist.append(round(self.irepa.euclid(state, end), 2))

    def callback(self, status, resp):
        self.times.append(round(resp.time, 2))


if __name__ == '__main__':
    end = np.array((12, 4, 0))
    ltest = LatencyTest()
    ltest.test(end)
    euclid_dist_arr = np.array(ltest.euclid_dist)
    calc_times_arr = np.array(ltest.calc_times)
    times_arr = np.array(ltest.times)

    plt.plot(euclid_dist_arr, calc_times_arr,
             marker='.', linestyle='', label='Calculation times')
    plt.xlabel('euclidian dist')
    plt.ylabel('time (s)')
    plt.legend()
    plt.show()
    plt.plot(euclid_dist_arr, times_arr,
             marker='.', linestyle='', label='Trajectory times')
    plt.xlabel('euclidian dist')

    plt.plot(euclid_dist_arr, times_arr - calc_times_arr,
             marker='.', linestyle='', label='Traj - Calc times')
    plt.axhline(y=0, color='black', linestyle='-')
    plt.xlabel('euclidian dist')
    plt.ylabel('time (s)')
    plt.legend()
    plt.show()

    plt.plot(times_arr,  calc_times_arr,
             marker='.', linestyle='')
    plt.xlabel('Calculation time (s)')
    plt.ylabel('Trajectory time (s)')
    plt.legend()
    plt.show()

    rospy.spin()

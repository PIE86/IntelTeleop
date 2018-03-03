#!/usr/bin/env python

import random
import time
import numpy as np
import rospy
import actionlib
from roadmap.msg import OptControlAction, OptControlGoal
from opt_control.srv import Samples
from prm_graph import PRM
from networks import Dataset, Networks

OPT_CONTROL_ACTION_SERVER = 'solve_ocp'
SAMPLING_SERVICE = 'create_samples'

VERBOSE = False

# --- HYPER PARAMS
# True PRM should be computed, False if  loaded from file
INIT_PRM = True
# Number of total iteration of the IREPA
IREPA_ITER = 4
NB_SAMPLE = 20
# NB_CONNECT = 3
# Densify longer
# NB_ATTEMPS_DENSIFY_LONGER = 10
# MIN_PATH_LEN = 3

# connexify
# NB_ATTEMPT_PER_CONNEX_PAIR = 5

# TODO: To get from Model node
NX = 3
NU = 2

# Range of the variables
X_MIN = 0 * np.ones(NX)
X_MAX = 20 * np.ones(NX)
U_MIN = -3 * np.ones(NU)
U_MAX = 3 * np.ones(NU)

random.seed(42)


class Irepa:

    def __init__(self, ocp_client, sampling_client):
        self.ocp_client = ocp_client
        self.sampling_client = sampling_client
        # Define an estimator
        self.estimator = Networks(NX, NU,
                                  x_range=np.array([X_MIN, X_MAX]),
                                  u_range=np.array([U_MIN, U_MAX]))

    def irepa_algo(self):

        # Initialize PRM with a sampling function,
        # a connect function and an heuristic distance
        prm = PRM(sample_fun=self.sample, connect_fun=self.connect,
                  hdistance=self.euclid)

        # Add NN_SAMPLE random nodes to the PRM
        prm.add_nodes(NB_SAMPLE, verbose=VERBOSE)
        # prm.densify_knn(NB_CONNECT)

        print('PRM initialized,', len(prm.graph.nodes), 'nodes')

        # Try to connect the nearest neighbors in the PRM
        # prm.connexify(None, NB_ATTEMPT_PER_CONNEX_PAIR)
        # prm.densify_longer_traj(NB_ATTEMPS_DENSIFY_LONGER, MIN_PATH_LEN)
        # prm.densify_longer_traj()

        i = 0
        stop = False
        while not stop and i < IREPA_ITER:
            print((('--- IREPA %d ---' % i)+'---'*10+'\n')*3, time.ctime())

            # Expand PRM
            # -----------------
            # Pick a pair of unconnected nearest neighbors
            # if distance > visibility horizon: # equivalent to longer_traj
            #   p* <- shortest path in PRM
            #   E <- ACADO(init = p*)
            # else: # equivalent to densify knn
            #   E <- ACADO(init = 0 or estimator)
            print('\n\n\n######################')
            print('EXPAND')
            prm.expand(first=(not bool(i)))
            print()
            print('Edge number:', len(prm.graph.edges))
            print('######################\n\n\n')

            stop = prm.is_fully_connected()

            # Build a dataset of subtrajectories
            # to train the estimator
            dset = Dataset(prm.graph)

            # Train the estimator on the dataset
            self.estimator.train(dset)

            # Test the estimator networks
            metrics = self.estimator.test(dset)
            print('\n##########')
            print('TEST ESTIMATOR')
            print('    value', metrics[0])
            print('    controls', metrics[1])
            print('    value', metrics[2])
            print('##########\n')

            # Improve the PRM where the estimator
            # gives better results
            print('\n\n\n######################')
            print('IMPROVE')
            stop = prm.improve(self.estimator)
            # returns False if estimator did better
            # than PRM

            i += 1

            # test
            print()
            print()
            print("\nEstimations at iteration", i)
            # test_traj_idx = random.sample(range(len(dataset.us)), 1)
            test_traj_idx = 18
            print('Dataset size:', len(dset.x1s), 'trajectories')
            x0 = dset.x1s[test_traj_idx, :].T
            x1 = dset.x2s[test_traj_idx, :].T
            print('x0 x1')
            print(x0)
            print(x1)
            print('Nets trajectories')
            X, U, V = self.estimator.trajectories(x0, x1)
            print('State trajectory')
            print(X)
            print('Control trajectory')
            print(U)
            print('Value')
            print(V)
            print('Euclidian value')
            print(self.euclid(x0, x1))

        self.estimator.save()

    def connect(self, s1, s2, init=None):
        """Tries to connect 2 sets by calling the Acado optimizer service.
        If init trajectory is passed, warm start of the optimization process"""
        print('Try to connect', s1, s2)

        if init is not None:
            X_init, U_init, V_init = init
            print('Using initialization, value:', V_init,
                  ', length:', X_init.shape[0])
            X_init = X_init.flatten()
            U_init = U_init.flatten()
        else:
            X_init, U_init, V_init = [], [], 0

        # res = opt_control_proxy(s1, s2, states, controls, cost)
        goal = OptControlGoal(
            s1, s2, X_init, U_init, V_init, NX, NU)
        res = self.ocp_client.send_goal(goal)
        self.ocp_client.wait_for_result()
        res = self.ocp_client.get_result()

        if res.success:
            print('  SUCCESS of optimization, time:', res.time,
                  'Path length:', len(res.states)//NX)
            X = np.array(res.states).reshape(len(res.states)//NX, NX)
            U = np.array(res.controls).reshape(len(res.controls)//NU, NU)
            return res.success, X, U, res.time
        else:
            print('  FAILURE of optimization')
            return res.success, [], [], 0

    def connect_test(self, s1, s2, init=None):
        success = random.randint(0, 1)

        trajlength = random.randint(10, 30)

        sxarr = np.array([s1[0], s2[0]])
        syarr = np.array([s1[1], s2[1]])
        sthetaarr = np.array([s1[2], s2[2]])

        Xx = np.linspace(s1[0], s2[0], trajlength)
        Xy = np.interp(Xx, sxarr, syarr)
        Xtheta = np.interp(Xx, sxarr, sthetaarr)

        X = np.vstack([Xx, Xy, Xtheta]).T
        U = X.copy()[:, 0:2]
        V = self.euclid(s1, s2) + 0.02*random.random()

        return success, X, U, V

    # def sample_test(self):
    #     return (round(random.uniform(0, 10), 3),
    #             round(random.uniform(0, 5), 3),
    #             round(random.uniform(0, 2*np.pi), 3))

    def sample(self, n):
        """n: number of samples to be returned"""
        resp = self.sampling_client(n)
        return np.array(resp.samples).reshape(n, int(len(resp.samples)/n))

    def euclid(self, s1, s2):
        #   print(s1, s2)
        return np.sqrt(sum((x1i - x2i)**2 for (x1i, x2i) in zip(s1, s2)))


if __name__ == '__main__':
    rospy.init_node('irepa_node')

    ocp_client = actionlib.SimpleActionClient(
                            OPT_CONTROL_ACTION_SERVER, OptControlAction)
    ocp_client.wait_for_server()
    rospy.loginfo('End of wait for ocp action server')

    rospy.wait_for_service('create_samples')
    sampling_client = rospy.ServiceProxy('create_samples', Samples)

    irepa = Irepa(ocp_client, sampling_client)
    irepa.irepa_algo()

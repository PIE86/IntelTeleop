#!/usr/bin/env python

import time
import random
import numpy as np
import matplotlib.pyplot as plt
import rospy
import actionlib
from roadmap.msg import OptControlAction, OptControlGoal
from opt_control.srv import Samples
from prm_graph import PRM
from networks import Dataset, Networks

OPT_CONTROL_ACTION_SERVER = 'solve_ocp'
SAMPLING_SERVICE = 'create_samples'

VERBOSE = False

# Number of total iteration of the IREPA
IREPA_ITER = 7
NB_SAMPLE = 35  # must be at least 5
SAVE = True  # save the NN weights at the end
PLOT = False  # plot the iterations

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
    """Irepa algorithm as described in Nicolas Mansard's paper."""

    def __init__(self):
        """
        Create ROS clients and a the estimator model
        """
        self.ocp_client = actionlib.SimpleActionClient(
            OPT_CONTROL_ACTION_SERVER, OptControlAction)
        self.ocp_client.wait_for_server()
        rospy.loginfo('End of wait for ocp action server')
        rospy.wait_for_service('create_samples')
        self.sampling_client = rospy.ServiceProxy('create_samples', Samples)

        self.estimator = Networks(NX, NU,
                                  x_range=np.array([X_MIN, X_MAX]),
                                  u_range=np.array([U_MIN, U_MAX]))

    def irepa_algo(self):
        """
        Build the PRM and Neural Networks then start the IREPA loop.
        """
        tstart = time.time()

        # Initialize PRM with a sampling function,
        # a connect function and an heuristic distance
        prm = PRM(sample_fun=self.sample, connect_fun=self.connect,
                  hdistance=euclid)

        # Add NN_SAMPLE random nodes to the PRM
        prm.add_nodes(NB_SAMPLE, verbose=VERBOSE)

        print('PRM initialized,', len(prm.graph.nodes), 'nodes')

        # For plotting purposes
        astar_successes = np.zeros(IREPA_ITER)
        est_successes = np.zeros(IREPA_ITER)
        nb_attempts = np.zeros(IREPA_ITER)
        edge_numbers_arr = np.zeros(IREPA_ITER)
        total_edges_cost_before = np.zeros(IREPA_ITER)
        total_edges_cost_after = np.zeros(IREPA_ITER)

        # Loop control variables
        i = 0
        stop = False
        while not stop and i < IREPA_ITER:
            print('\n'*5)
            print('################')
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
            nb_astar, nb_est, nb_attempt = prm.expand(self.estimator,
                                                      first=(not bool(i)))

            astar_successes[i] = nb_astar
            est_successes[i] = nb_est
            nb_attempts[i] = nb_attempt
            print()
            print('Edge number:', len(prm.graph.edges))
            edge_numbers_arr[i] = len(prm.graph.edges)
            print('######################\n\n\n')

            stop = prm.is_fully_connected()

            # Build a dataset of subtrajectories to train the estimator
            dset = Dataset(prm.graph)

            # Train the estimator on the dataset
            self.estimator.train(dset)

            # Improve the PRM when the estimator gives better results
            print('\n\n\n######################')
            print('IMPROVE')
            # Return true if on at least one of the edges the estimator was
            # better than the PRM and ACADO initialized by this Estimations
            # also did better thant the PRM
            total_edges_cost_before[i] = prm.graph.total_cost()
            stop = prm.improve(self.estimator)
            total_edges_cost_after[i] = prm.graph.total_cost()

            i += 1

            # Test the estimation of a random trajectory (not useful)
            # self.test(dset, i)

        tend = time.time()
        print('\n##############')
        print('IREPA was executed in ', (tend-tstart)/60, 'minutes')
        print()

        if SAVE:
            print('Saving estimator weights')
            self.estimator.save()
            print('Saved')

        if PLOT:
            self.plot_results(astar_successes, est_successes, nb_attempts,
                              edge_numbers_arr, total_edges_cost_before,
                              total_edges_cost_after)

    def connect(self, s1, s2, init=None):
        """
        Tries to connect 2 states by calling the Acado optimizer service.
        If init trajectory is passed, warm start of the optimization process
        using initialization trajectories.

        :param s1: start state
        :param s2: start state
        :param init: tuple containing 3 fields
                     - states trajectory
                     - controls trajectory
                     - value of the trajectory
        :type s1: numpy.array
        :type s1: numpy.array
        :type init: tuple of size 3 (numpy.array, numpy.array, float)
        """
        print('Try to connect', s1, s2)

        if init is not None:
            X_init, U_init, V_init = init
            print('Using initialization, value:', V_init,
                  ', length:', X_init.shape[0])
            X_init = X_init.flatten()
            U_init = U_init.flatten()
        else:
            X_init, U_init, V_init = [], [], 0

        goal = OptControlGoal(
            s1, s2, X_init, U_init, V_init, NX, NU)
        res = self.ocp_client.send_goal(goal)
        # Force waiting for results to avoid overloading acado solver
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
        """
        Legacy dummy function. Was used instead of connect when acado node
        was not implemented. Trajectories are calculated using linear
        interpolation and value using euclidian distance.

        :param s1: start state
        :param s2: start state
        :param init: tuple containing 3 fields
                     - states trajectory
                     - controls trajectory
                     - value of the trajectory
        :type s1: numpy.array
        :type s1: numpy.array
        :type init: tuple of size 3 (numpy.array, numpy.array, float)
        """
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
        V = euclid(s1, s2) + 0.02*random.random()

        return success, X, U, V

    def sample(self, n):
        """
        Use the sampling client to generate n samples from the acceptable
        state space.

        :param n: number of samples to be returned
        :type n: int
        """
        resp = self.sampling_client(n)
        return np.array(resp.samples).reshape(n, int(len(resp.samples)/n))

    def plot_results(self, astar_successes, est_successes, nb_attempts,
                     edge_numbers_arr, total_edges_cost_before,
                     total_edges_cost_after):

        """
        Plot the results of the IREPA loop.

        :param astar_successes: number of successfully calculated trajectories
                                at each iteration using astar initialization
        :param est_successes: number of successfully calculated trajectories
                              at each iteration using estimator initialization
        :param nb_attempts: number of connection attempted at each iteration
                            (in fact, number of remaining edges to create)
        :param edge_numbers_arr: number of edges in the graph at each iteration
        :param total_edges_cost_before: sum of all edges costs before the
                                        improve step
        :param total_edges_cost_after: sum of all edges costs after the
                                       improve step
        """
        MAX_EDGE_NB = NB_SAMPLE * (NB_SAMPLE - 1)
        iters = np.arange(IREPA_ITER, dtype=np.int64)

        plt.plot(iters, astar_successes,
                 color='blue', label='astar',
                 marker='.', markersize=15, linestyle='dashed')
        plt.plot(iters, est_successes,
                 color='green', label='estimator',
                 marker='.', markersize=15, linestyle='dashed')
        plt.plot(iters, nb_attempts,
                 color='orange', label='attempts',
                 marker='.', markersize=15, linestyle='dashed')
        plt.xticks(iters)
        plt.legend()
        plt.show()

        plt.axhline(y=MAX_EDGE_NB, color='black', linestyle='dashed')
        plt.plot(iters, edge_numbers_arr,
                 color='green', label='attempts',
                 marker='.', markersize=15, linestyle='dashed')
        plt.xticks(iters)
        plt.legend()
        plt.show()

        plt.plot(iters, total_edges_cost_before,
                 color='gray', label='total cost before',
                 marker='.', markersize=15, linestyle='dashed')
        plt.plot(iters, total_edges_cost_after,
                 color='green', label='total cost after',
                 marker='.', markersize=15, linestyle='dashed')
        plt.xticks(iters)
        plt.legend()
        plt.show()

    def test(self, dset, i):
        # Test the estimator networks
        metrics = self.estimator.test(dset)
        print('\n##########')
        print('TEST ESTIMATOR')
        print('    value', metrics[0])
        print('    controls', metrics[1])
        print('    value', metrics[2])
        print('##########\n')

        print()
        print("\nEstimations at iteration", i)
        test_traj_idx = random.sample(range(len(dset.us)), 1)
        # test_traj_idx = 18
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
        print(euclid(x0, x1))


def euclid(s1, s2):
    """
    Compute euclidian distance between 2 states (numpy arrays)
    """
    return np.sqrt(sum((x1i - x2i)**2 for (x1i, x2i) in zip(s1, s2)))


if __name__ == '__main__':
    rospy.init_node('irepa_node')
    irepa = Irepa()
    irepa.irepa_algo()

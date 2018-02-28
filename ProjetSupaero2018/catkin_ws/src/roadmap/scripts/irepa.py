#!/usr/bin/env python

import random
import time
import numpy as np
import rospy
from opt_control.srv import OptControl

from prm_graph import PRM
from networks import Dataset, Networks

OPT_CONTROL_SERVICE = 'solve_ocp'

rospy.init_node('irepa_node')
rospy.wait_for_service(OPT_CONTROL_SERVICE)
rospy.loginfo('End of wait for rocket')
opt_control_proxy = rospy.ServiceProxy(OPT_CONTROL_SERVICE, OptControl)


VERBOSE = False

# --- HYPER PARAMS
# True PRM should be computed, False if  loaded from file
INIT_PRM = True
# Number of total iteration of the IREPA
IREPA_ITER = 4
NB_SAMPLE = 8
NB_CONNECT = 3
# Densify longer
NB_ATTEMPS_DENSIFY_LONGER = 10
MIN_PATH_LEN = 3

# connexify
NB_ATTEMPT_PER_CONNEX_PAIR = 5

# TODO: To get from Model node
STATE_SIZE = 3
CONTROL_SIZE = 2

random.seed(42)


def irepa():

    # Initialize PRM with a sampling function,
    # a connect function and an heuristic distance
    # prm = PRM(sample_fun=sample, connect_fun=connect_test, hdistance=euclid)
    prm = PRM(sample_fun=sample, connect_fun=connect, hdistance=euclid)

    # Add NN_SAMPLE random nodes to the PRM
    prm.add_nodes(NB_SAMPLE, verbose=VERBOSE)
    # prm.densify_knn(NB_CONNECT)

    print('PRM initialized,', len(prm.graph.nodes), 'nodes')

    # Define an estimator
    estimator = Networks(STATE_SIZE, CONTROL_SIZE)

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
        dataset = Dataset(prm.graph)

        # Train the estimator on the dataset
        estimator.train(dataset)

        # Improve the PRM where the estimator
        # gives better results
        stop = prm.improve(estimator)
        # returns False if estimator did better
        # than PRM

        i += 1

        # test
        print()
        print()
        print("\nEstimations at iteration", i)
        # test_traj_idx = random.sample(range(len(dataset.us)), 1)
        test_traj_idx = 18
        print('Dataset size:', len(dataset.x1s), 'trajectories')
        x0 = dataset.x1s[test_traj_idx, :].T
        x1 = dataset.x2s[test_traj_idx, :].T
        print('x0 x1')
        print(x0)
        print(x1)
        print('Nets trajectories')
        X, U, V = estimator.trajectories(x0, x1)
        print('State trajectory')
        print(X)
        print('Control trajectory')
        print(U)
        print('Value')
        print(V)
        print('Euclidian value')
        print(euclid(x0, x1))


def connect(s1, s2, init=None):
    """Tries to connect 2 sets by calling the Acado optimizer service.
    If init trajectory is passed, warm start of the optimization process"""
    print('Try to connect', s1, s2)
    p1 = s1
    p2 = s2

    if init is not None:
        X_init, U_init, V_init = init
        print('Using initialization, value:', V_init, ', length:', X_init.shape[0])
        X_init = X_init.flatten()
        U_init = U_init.flatten()
    else:
        X_init, U_init, V_init = [], [], 0

    # resp = opt_control_proxy(p1, p2, states, controls, cost)
    resp = opt_control_proxy(p1, p2, X_init, U_init, V_init, STATE_SIZE, CONTROL_SIZE)

    if resp.success:
        print('  SUCCESS of optimization, time:', resp.time, 'Path length:', len(resp.states)//STATE_SIZE)
        X = np.array(resp.states).reshape(len(resp.states)//STATE_SIZE, STATE_SIZE)
        U = np.array(resp.controls).reshape(len(resp.controls)//CONTROL_SIZE, CONTROL_SIZE)
        return resp.success, X, U, resp.time
    else:
        print('  FAILURE of optimization')
        return resp.success, [], [], 0


# Placeholder
def connect_test(s1, s2, init=None):
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


def sample():
    return (round(random.uniform(0, 10), 3),
            round(random.uniform(0, 5), 3),
            round(random.uniform(0, 2*np.pi), 3))


def euclid(s1, s2):
    #   print(s1, s2)
    return np.sqrt(sum((x1i - x2i)**2 for (x1i, x2i) in zip(s1, s2)))


if __name__ == '__main__':
    irepa()

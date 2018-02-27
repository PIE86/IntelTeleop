#!/usr/bin/env python

import random
import time
import numpy as np
import rospy
from opt_control.srv import OptControl
from geometry_msgs.msg import Point

from prm_graph import PRM
from networks import Dataset, Networks

OPT_CONTROL_SERVICE = 'solve_rocket'

rospy.init_node('irepa_node')
rospy.wait_for_service('solve_rocket')
rospy.loginfo('End of wait for rocket')
opt_control_proxy = rospy.ServiceProxy('solve_rocket', OptControl)


VERBOSE = False

# --- HYPER PARAMS
# True PRM should be computed, False if  loaded from file
INIT_PRM = True
# Number of total iteration of the IREPA
IREPA_ITER = 3
NB_SAMPLE = 30
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
    print("\n Final value of estimated X trajectory:")
    batch = random.sample(range(len(dataset.us)), 1)
    print('Dataset size:', len(dataset.x1s), 'trajectories')
    x0 = dataset.x1s[batch, :].T
    x1 = dataset.x2s[batch, :].T
    print('x0 x1')
    print(x0)
    print(x1)
    print('Nets connect_test')
    print(estimator.connect_test(x0, x1))
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
    p1 = Point(*s1)
    p2 = Point(*s2)

    if init is not None:
        print('Using initialization')
        X_init, U_init, V_init = init
        X_init = [Point(*s) for s in X_init]
        U_init = [Point(u[0], u[1], 0) for u in U_init]
    else:
        X_init, U_init, V_init = [], [], 0

    # resp = opt_control_proxy(p1, p2, states, controls, cost)
    resp = opt_control_proxy(p1, p2, X_init, U_init, V_init)
    print('Path length:', len(resp.states))

    if resp.success:
        print('  SUCCESS of optimization, time:', resp.time)
        X = np.array([[s.x, s.y, s.z] for s in resp.states])
        U = np.array([[u.x, u.y] for u in resp.states])
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

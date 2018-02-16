import random
import time
import numpy as np
from prm_graph import PRM
from networks import Dataset, Networks


# --- HYPER PARAMS
# True PRM should be computed, False if  loaded from file
INIT_PRM = True
# Number of total iteration of the IREPA
IREPA_ITER = 5
NB_SAMPLE = 15
NB_CONNECT = 3

# TODO: To get from Model node
STATE_SIZE = 3
CONTROL_SIZE = 2

random.seed(42)


def irepa():
    prm = PRM(sample, connect_test)
    prm.add_nodes(NB_SAMPLE)
    prm.densify_knn(euclid, NB_CONNECT)

    print('PRM initialized')
    print(len(prm.graph.nodes), 'nodes:')
    print(list(prm.graph.nodes), '\n')
    print(len(prm.graph.edges), 'edges:')
    print(list(prm.graph.edges), '\n')

    prm.connexify(None, 5)
    prm.densify_longer_traj(euclid)

    # test
    nets = Networks(STATE_SIZE, CONTROL_SIZE)
    print("\n Initial value of estimated X trajectory:")
    dataset = Dataset(prm.graph)
    batch = random.sample(range(len(dataset.us)), 1)
    x0 = dataset.x1s[batch, :].T
    x1 = dataset.x2s[batch, :].T

    print(nets.connect_test(x0, x1))

    # dataset = Dataset(prm.graph)
    for i in range(IREPA_ITER):
        print((('--- IREPA %d ---' % i)+'---'*10+'\n')*3, time.ctime())
        dataset = Dataset(prm.graph)
        nets.train(dataset)
        prm.improve(nets)

    # test
    print("\n Final value of estimated X trajectory:")
    batch = random.sample(range(len(dataset.us)), 1)
    x0 = dataset.x1s[batch, :].T
    x1 = dataset.x2s[batch, :].T
    print(nets.connect_test(x0, x1))


def connect(s1, s2, init=None):
    """Tries to connect 2 sets by calling the Acado optimizer service.
    If init trajectory is passed, warm start of the optimization process"""
    pass


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
    V = euclid(s1, s2)

    return success, X, U, V


def sample():
    return random.randint(1, 10), random.randint(1, 10), random.randint(1, 10)


def euclid(s1, s2):
    print(s1, s2)
    return np.sqrt(sum((x1i - x2i)**2 for (x1i, x2i) in zip(s1, s2)))


if __name__ == '__main__':
    irepa()

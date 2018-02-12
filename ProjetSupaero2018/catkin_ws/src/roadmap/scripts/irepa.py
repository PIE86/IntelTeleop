import random
import time
import numpy as np
from prm_graph import PRM
from networks import Dataset, Networks


# --- HYPER PARAMS
# True PRM should be computed, False if  loaded from file
INIT_PRM = False
# Number of total iteration of the IREPA
IREPA_ITER = 5
NB_SAMPLE = 3
NB_CONNECT = 3

# TODO: To get from Model node
STATE_SIZE = 3
CONTROL_SIZE = 2


def irepa():
    nets = Networks(STATE_SIZE, CONTROL_SIZE)
    prm = PRM(sample, connect, NB_SAMPLE, NB_CONNECT)
    prm.build_graph(euclid)
    print('PRM initialized')
    print(len(prm.graph.nodes), 'nodes')
    print(len(prm.graph.edges), 'edges')
    # TODO: Connexify PRM -> nodes couples tried several times?
    # TODO: Densify PRM
    # dataset = Dataset(prm.graph)
    for i in range(IREPA_ITER):
        print((('--- IREPA %d ---' % i)+'---'*10+'\n')*3, time.ctime())
        dataset = Dataset(prm.graph)
        nets.train(dataset)
        prm.improve(nets)


# TODO: another file
def connect(s1, s2):
    """Send a request to Acado optimizer service.
    Warm start argument?
    """
    success = True
    X = np.vstack([(1, 6, 4), (2, 9, 9), (6, 5, 1), (1, 6, 4), (2, 9, 9),
                   (1, 6, 4), (2, 9, 9), (6, 5, 1), (1, 6, 4), (2, 9, 9),
                   (1, 6, 4), (2, 9, 9), (6, 5, 1), (1, 6, 4), (2, 9, 9),
                   ])
    U = np.vstack([(1, 6), (5, 2), (4, 1), (1, 6), (5, 2),
                  (1, 6), (5, 2), (4, 1), (1, 6), (5, 2),
                  (1, 6), (5, 2), (4, 1), (1, 6), (5, 2),
                   ])
    T = 10
    return success, X, U, T


def sample():
    return random.randint(1, 10), random.randint(1, 10), random.randint(1, 10)


def euclid(s1, s2):
    return np.sqrt(sum((x1i - x2i)**2 for (x1i, x2i) in zip(s1, s2)))


if __name__ == '__main__':
    irepa()

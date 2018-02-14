import random
import numpy as np
from numpy.linalg import norm as npnorm
import tensorflow as tf
import tflearn


'''
Implementation of the networks trained from the dataset.  The networks are used
to approximate 3 functions: the value function V(a,b) which is the minimal cost
to pay for going from a to b ; the X- ; and the U-trajectories X(a,b) and
U(a,b) which are the state and control trajectories to go from a to b.
'''

# boundaries of the search space
xmin = np.array([1, 1, 1])
xmax = np.array([10, 10, 10])
umax = np.array([2, 3])


class Networks:
    BATCH_SIZE = 128

    def __init__(self, state_size, control_size):
        self.TRAJLENGTH = 20
        # bx = [10., 10., 10., 1.4, 1.4, 10., 10., 10., 2., 2.]
        # bx = bx * TRAJLENGTH
        # bx = [[-x for x in bx], bx]
        # vstack/hstack: stack several arrays of same dims as column/line
        # Represent the states composants boundaries?
        self.state_size = state_size
        self.control_size = control_size
        bx = np.vstack([np.hstack([xmin, xmax])] * self.TRAJLENGTH).T
        self.bx = bx

        # 2 is for the 2 concatenated states: beginning and end
        self.value = NN(state_size * 2, 1).setupOptim('direct')
        self.ptrajx = NN(state_size * 2, state_size * self.TRAJLENGTH,
                         umax=bx).setupOptim('direct')
        self.ptraju = NN(state_size * 2, control_size * self.TRAJLENGTH,
                         umax=umax).setupOptim('direct')

        self.sess = tf.InteractiveSession()
        tf.global_variables_initializer().run()

    def train(self, dataset, nets=None, nepisodes=int(1e2), track=True):
        if nets is None:
            nets = [self.value, self.ptrajx, self.ptraju]

        if track:
            hist = []
            refbatch = random.choices(
                range(len(dataset.us)), k=self.BATCH_SIZE*16)

            xref = np.hstack([dataset.x1s[refbatch, :],
                              dataset.x2s[refbatch, :]])
            vref = dataset.vs[refbatch, :]
            xsref = dataset.trajxs[refbatch, :]
            usref = dataset.trajus[refbatch, :]

        for episode in range(nepisodes):
            batch = random.choices(
                range(len(dataset.us)), k=self.BATCH_SIZE*16)
            xbatch = np.hstack([dataset.x1s[batch, :],
                                dataset.x2s[batch, :]])

            # FIXME
            """
            ValueError: Cannot feed value of shape (2048, 60)
            for Tensor 'Placeholder_2:0', which has shape '(?, 40)'
            """
            self.sess.run([p.optim for p in nets],
                          feed_dict={
                self.value.x: xbatch,
                self.value.uref: dataset.vs[batch, :],
                self.ptrajx.x: xbatch,
                self.ptrajx.uref: dataset.trajxs[batch, :],
                self.ptraju.x: xbatch,
                self.ptraju.uref: dataset.trajus[batch, :]})

            if track and not episode % 50:
                v = self.sess.run(self.value.policy,
                                  feed_dict={self.value.x: xref})
                xs = self.sess.run(self.ptrajx.policy,
                                   feed_dict={self.ptrajx.x: xref})
                us = self.sess.run(self.ptraju.policy,
                                   feed_dict={self.ptraju.x: xref})
                hist.append([npnorm(v - vref) / len(refbatch),
                             npnorm(us - usref) / len(refbatch),
                             npnorm(xs - xsref) / len(refbatch)])
                # print(npnorm(v - vref) / len(refbatch))

        if track:
            return hist

    def trajectories(self, x1=None, x2=None):
        '''Returns a triplet X,U,V (ie a vector sampling the time function) to go
        from x0 to x1, computed from the networks (global variable).'''
        # print(x1)
        # print(x2)
        x = np.hstack([x1, x2]).reshape((1, 2*self.state_size))
        X = self.sess.run(self.ptrajx.policy, feed_dict={self.ptrajx.x: x})
        U = self.sess.run(self.ptraju.policy, feed_dict={self.ptraju.x: x})
        U = U.reshape((self.TRAJLENGTH, self.control_size))
        V = self.sess.run(self.value.policy, feed_dict={self.value.x: x})[0, 0]

        return X, U, V

    def connect_test(self, x0, x1):
        return self.ptrajx.predict(self.sess, x0, x1)

    def connect(self, x0, x1):
        pass


UPDATE_RATE = 5e-3


class NN:

    '''
    Simple network from X state to U state, where X and U may or may not have
    the same dimension.
    '''

    def __init__(self, input_size, ouput_size, nb_layer1=250, nb_layer2=None,
                 umax=None):
        if nb_layer2 is None:
            nb_layer2 = nb_layer1
        self.input_size = input_size
        self.output_size = ouput_size
        winit = WeightInit(1)

        nvars = len(tf.trainable_variables())
        olay = 'relu' if umax is None else 'tanh'

        x = tflearn.input_data(shape=[None, input_size])
        net = tflearn.fully_connected(
            x,  nb_layer1, activation='relu', weights_init=winit.n)
        net = tflearn.fully_connected(
            net, nb_layer2, activation='relu', weights_init=winit.n)
        policy = tflearn.fully_connected(
            net, ouput_size,  activation=olay, weights_init=winit.u)

        self.x = x  # Network input <x> in Pi(x)
        # TODO: policy is a bad name
        self.policy = policy  # Network output <Pi>
        self.variables = tf.trainable_variables()[
            nvars:]       # Variables to be trained

        self.withUmax(umax)

    def withUmax(self, umax):
        if umax is None:
            print('umax none')
            return
        umax = np.matrix(umax)
        self.head = self.policy
        if umax.shape == (1, 1):
            print('umax float')
            self.policy = self.policy * umax[0, 0]
        elif umax.shape == (1, 2):
            print('umax pair')
            self.policy = (
                self.policy * (umax[0, 1]
                               - umax[0, 0]) + umax[0, 0] + umax[0, 1]) / 2
        elif umax.shape == (2, self.output_size):
            print('umax list')
            l, u = umax
            self.policy = (tf.multiply(self.policy, u - l) + l + u) / 2

    def setupOptim(self, otype='actorcritic'):
        if otype == 'actorcritic':
            return self.setupActorCriticOptim()
        else:
            return self.setupDirectOptim()

    def setupActorCriticOptim(self, learningRate=UPDATE_RATE):
        qgradient = tf.placeholder(tf.float32, [None, self.output_size])
        grad = tf.gradients(self.policy, self.variables, -qgradient)
        optim = tf.train.AdamOptimizer(learningRate).\
            apply_gradients(zip(grad, self.variables))

        self.qgradient = qgradient  # Qvalue gradient wrt control (input value)
        self.optim = optim  # Optimizer
        return self

    def setupDirectOptim(self, learningRate=UPDATE_RATE):
        uref = tf.placeholder(tf.float32, [None, self.output_size])
        loss = tflearn.mean_square(uref, self.policy)
        optim = tf.train.AdamOptimizer(learningRate).minimize(loss)

        self.optim = optim  # Optimizer
        self.uref = uref
        self.loss = loss
        return self

    def setupTargetAssign(self, nominalNet, tau=UPDATE_RATE):
        self.update_variables = \
            [target.assign(tau * ref + (1 - tau) * target)
             for target, ref in zip(self.variables, nominalNet.variables)]
        return self

    def predict(self, session, x0, x1):
        '''Returns a triplet X,U,V (ie a vector sampling the time function) to go
        from x0 to x1, computed from the networks (global variable).'''
        x0 = x0.T if x0 is not None else env.sample().V
        x1 = x1.T if x1 is not None else env.sample().V
        x = np.hstack([x0, x1])
        Y = session.run(self.policy, feed_dict={self.x: x})

        return Y


class Dataset:
    def __init__(self, graph):
        self.graph = graph
        self.indexes = []
        self.set()

    def set(self):
        x1s = []  # init points
        x2s = []  # term points
        vs = []  # values
        us = []  # controls
        trajxs = []  # trajs state
        trajus = []  # trajs state

        print('Load dataset ')
        # for every edge trajectory
        for (p1, p2), (X, U, V) in self.graph.edges.items():
            print('.', end='')
            DV = V / (len(X) - 1)
            # for every instant of the trajectory
            for k, (x1, u1) in enumerate(zip(X, U)):
                # Create subtrajectory of minimum size 7 to the end of the traj
                # resample this trajectory: if trajectory length < 20: more
                # points, otherwise less
                for di, x2 in enumerate(X[k+1:]):
                    if di < 5:
                        continue
                    x1s.append(x1)
                    x2s.append(x2)
                    us.append(u1)
                    vs.append(DV * (di + 1))
                    # np.ravel -> flatten any array in a 1D array

                    trajxs.append(np.ravel(resample(X[k:k+di+2], 20)))
                    trajus.append(np.ravel(resample(U[k:k+di+2], 20)))
                    self.indexes.append([p1, p2, k, di])

        print('\n')
        self.x1s = np.vstack(x1s)
        self.x2s = np.vstack(x2s)
        self.vs = np.vstack(vs)
        self.us = np.vstack(us)
        self.trajxs = np.vstack(trajxs)
        self.trajus = np.vstack(trajus)


class WeightInit:

    def __init__(self, seed):
        self.reset(seed)

    def reset(self, seed):
        self.seed = seed
        self.n = tflearn.initializations.truncated_normal(seed=seed)
        self.u = tflearn.initializations.uniform(minval=-0.003, maxval=0.003,
                                                 seed=seed)


def resample(X, N):
    """
    Resample in N iterations the trajectory X. The output is a
    trajectory similar to X with N points. Whatever the length of X (longer
    or shorter than N), the trajectories out are all of size N.
    """
    # TODO: Maybe better than a loop?
    # Number of points in the trajectory X
    nx = X.shape[0]
    idx = (np.arange(float(N)) / (N - 1)) * (nx - 1)
    hx = []
    for i in idx:
        i0 = int(np.floor(i))
        i1 = int(np.ceil(i))
        di = i % 1
        # barycenter of the to closest points in X
        x = X[i0, :] * (1 - di) + X[i1, :] * di
        hx.append(x)

    # could use np.array but sometimes better performances with vstack
    return np.vstack(hx)

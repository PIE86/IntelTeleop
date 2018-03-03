import sys
import random
import numpy as np
from keras.models import Sequential, load_model
from keras.layers.core import Dense, Dropout, Activation
from sklearn.preprocessing import StandardScaler


'''
Implementation of the networks trained from the dataset.  The networks are used
to approximate 3 functions: the value function V(a,b) which is the minimal cost
to pay for going from a to b ; the X- ; and the U-trajectories X(a,b) and
U(a,b) which are the state and control trajectories to go from a to b.
'''

TRAJLENGTH = 21


class Networks:
    BATCH_SIZE = 128

    def __init__(self, state_size, control_size, x_range, u_range):
        self.TRAJLENGTH = TRAJLENGTH
        self.state_size = state_size
        self.control_size = control_size

        self.value = self._create_model(state_size * 2, 1)
        self.ptrajx = self._create_model(state_size * 2,
                                         state_size * self.TRAJLENGTH)
        self.ptraju = self._create_model(state_size * 2,
                                         control_size * self.TRAJLENGTH)

        # Fit standard scalers with data ranges
        self.xs_scaler = StandardScaler().fit(np.tile(x_range, 2))
        self.x_scaler = StandardScaler().fit(np.tile(x_range, TRAJLENGTH))
        self.u_scaler = StandardScaler().fit(np.tile(u_range, TRAJLENGTH))

    def train(self, dset, nepisodes=int(1e2)):
        # TODO track
        batch = random.choices(
            range(len(dset.us)), k=self.BATCH_SIZE*16)

        xbatch = self.xs_scaler.transform(np.hstack([dset.x1s[batch, :],
                                                     dset.x2s[batch, :]]))

        self.value.fit(xbatch,
                       dset.vs[batch, :],
                       batch_size=self.BATCH_SIZE,
                       epochs=nepisodes, verbose=True)

        self.ptrajx.fit(xbatch,
                        self.x_scaler.transform(dset.trajxs[batch, :]),
                        batch_size=self.BATCH_SIZE,
                        epochs=nepisodes, verbose=False)

        self.ptraju.fit(xbatch,
                        self.u_scaler.transform(dset.trajus[batch, :]),
                        batch_size=self.BATCH_SIZE,
                        epochs=nepisodes, verbose=False)

    def test(self, dset):
        """Test over the whole dataset"""
        xbatch = self.xs_scaler.transform(np.hstack([dset.x1s, dset.x2s]))
        value_metrics = self.value.evaluate(
                                    xbatch,
                                    dset.vs,
                                    batch_size=self.BATCH_SIZE)
        states_metrics = self.ptrajx.evaluate(
                                    xbatch,
                                    self.x_scaler.transform(dset.trajxs),
                                    batch_size=self.BATCH_SIZE)
        controls_metrics = self.ptraju.evaluate(
                                    xbatch,
                                    self.u_scaler.transform(dset.trajus),
                                    batch_size=self.BATCH_SIZE)
        return value_metrics, states_metrics, controls_metrics

    def trajectories(self, x1, x2):
        """
        Returns a triplet X,U,V (ie a vector sampling the time function) to go
        from x0 to x1, computed from the networks (global variable).
        """
        try:
            x = self.xs_scaler.transform(np.hstack([x1, x2])
                                           .reshape((1, 2*self.state_size)))
        except ValueError as e:
            print(e)
            print()
            print()
            print()
            print()
            print(x1, x2)
            print(self.state_size)

        X = self.x_scaler.inverse_transform(
                    self.ptrajx.predict(x, batch_size=self.BATCH_SIZE))
        X = X.reshape((self.TRAJLENGTH, self.state_size))
        U = self.u_scaler.inverse_transform(
                    self.ptraju.predict(x, batch_size=self.BATCH_SIZE))
        U = U.reshape((self.TRAJLENGTH, self.control_size))
        V = self.value.predict(x, batch_size=self.BATCH_SIZE)
        return X, U, V

    def _create_model(self, input_size, output_size, nb_layer1=250,
                      nb_layer2=250):
        model = Sequential()
        model.add(Dense(nb_layer1, kernel_initializer='lecun_uniform',
                        input_shape=(input_size,)))
        model.add(Activation('relu'))
        model.add(Dropout(0.2))
        model.add(Dense(nb_layer2, kernel_initializer='lecun_uniform'))
        model.add(Activation('relu'))
        model.add(Dropout(0.2))
        model.add(Dense(output_size, kernel_initializer='lecun_uniform'))
        model.add(Activation('linear'))
        model.compile(loss='mse', optimizer="rmsprop")
        return model

    def connect_test(self, x1, x2):
        x = np.hstack([x1, x2]).reshape((1, 2*self.state_size))
        return self.ptrajx.predict(x, batch_size=self.BATCH_SIZE)

    def connect(self, x1, x2):
        pass

    def save(self):
        self.value.save("model_value.hd5")
        self.ptraju.save("model_ptraju.hd5")
        self.ptrajx.save("model_ptrajx.hd5")

    def load(self):
        try:
            self.value = load_model("model_value.hd5")
            self.ptraju = load_model("model_ptraju.hd5")
            self.ptrajx = load_model("model_ptrajx.hd5")
        except Exception:
            print()
            print()
            print("No weights saved")


class Dataset:
    def __init__(self, graph):
        self.graph = graph
        self.indexes = []
        self.set()

    def __str__(self):
        return '\n'.join([
            '##################',
            'Dataset:',
            'Number of X traj: ' + str(len(self.trajxs)),
            'Number of U traj: ' + str(len(self.trajus)),
        ])

    def set(self):
        x1s = []  # init points
        x2s = []  # term points
        vs = []  # values
        us = []  # controls
        trajxs = []  # trajs state
        trajus = []  # trajs state

        # TODO: LENT!
        print('Load dataset ')
        # for every edge trajectory
        nb_edges = len(self.graph.edges)
        i = 0
        for (p1, p2), (X, U, V) in self.graph.edges.items():
            i += 1
            # print progress in percents
            sys.stdout.write("\r{}%".format(round(100*float(i)/nb_edges, 0)))
            sys.stdout.flush()
            DV = V / (len(X) - 1)
            # for every instant of the trajectory
            for k, (x1, u1) in enumerate(zip(X, U)):
                # Create subtrajectory of minimum size 7 to the end of the traj
                # resample this trajectory: if trajectory length < 21: more
                # points, otherwise less
                for di, x2 in enumerate(X[k+1:]):
                    if di < 5:
                        continue
                    x1s.append(x1)
                    x2s.append(x2)
                    us.append(u1)
                    vs.append(DV * (di + 1))
                    # np.ravel -> flatten any array in a 1D array

                    trajxs.append(np.ravel(resample(X[k:k+di+2], TRAJLENGTH)))
                    trajus.append(np.ravel(resample(U[k:k+di+2], TRAJLENGTH)))
                    self.indexes.append([p1, p2, k, di])

        print('\n')
        self.x1s = np.vstack(x1s)
        self.x2s = np.vstack(x2s)
        self.vs = np.vstack(vs)
        self.us = np.vstack(us)
        self.trajxs = np.vstack(trajxs)
        self.trajus = np.vstack(trajus)


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

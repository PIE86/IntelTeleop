import random
from itertools import permutations
import numpy as np
from os.path import join as pjoin
import heapq

from collections import namedtuple


Path = namedtuple('Path', ['X', 'U', 'V'])
Node = namedtuple('Node', ['state', 'linked_to'])


class PRM:
    """
    Simple generic PRM implementation using the Graph structure

    sample: generate a random state in the admissible state space
    connect: determines if 2 states are connectable and return the edges
             between the 2 nodes
    """

    def __init__(self, sample_fun, connect_fun, hdistance):
        self.sample = sample_fun
        self.ACADO_connect = connect_fun
        self.graph = Graph(hdistance)
        self.visibility_horizon = 5

    def add_nodes(self, nb_sample=40, verbose=True):
        """Add a given number of nodes"""
        # TODO: add an option to "guide this adding"
        # Ex: add nodes between 2 unconnectable nodes...
        for _ in range(nb_sample):

            state = self.sample()
            node_index = self.graph.add_node(state)

    def expand(self, first=False):
        """ Expand PRM
        -----------------
        Pick a pair of unconnected nearest neighbors
        if distance > visibility horizon: # equivalent to longer_traj
          p* <- shortest path in PRM
          E <- ACADO(init = p*)
        else: # equivalent to densify knn
          E <- ACADO(init = 0 or estimator)
        """

        for (node1, node2), distance in zip(*self.unconnected_2_nn()):

            if distance > self.visibility_horizon:
                if first:
                    print(node1, node2, 'too far')
                    continue
                path = self.graph.get_path(node1, node2)

            else:
                # TODO: path = estimator.predict(X)
                path = None  # or estimator

            success, X, U, V = self.ACADO_connect(
                self.graph.nodes[node1].state,
                self.graph.nodes[node2].state,
                init=path)

            # If successing while attempting to connect the two nodes
            # then add the new edge to the graph
            if success:
                self.graph.add_edge((node1, node2), X, U, V)

    def unconnected_2_nn(self):
        unconnected_pairs = [
            pair for pair in permutations(list(self.graph.nodes.keys()), 2)
            if pair not in self.graph.edges
        ]
        distance_list = [
            self.graph.hdistance(self.graph.nodes[pair[0]].state,
                                 self.graph.nodes[pair[1]].state)
            for pair in unconnected_pairs
        ]
        print('NUMBER OF UNCONNECTED PAIRS:', len(unconnected_pairs))

        return unconnected_pairs, distance_list

    def is_fully_connected(self):
        nb_nodes, nb_edges = len(self.graph.nodes), len(self.graph.edges)
        return nb_edges == nb_nodes*(nb_nodes-1)

    def densify_knn(self, hdistance, nb_connect=3, nb_best=None):
        """Build the prm graph
        nb_sample: number of samples taken from the state space
        nb_connect: number of nearest state to try to connect with
        nb_best: number of the best edges to keep
        """
        new_edges = []
        for node_index, node in self.graph.nodes.items():
            nearest_nodes = self.graph.nn_from_state(node.state, hdistance,
                                                     nb_connect)
            for nn_node_index in nearest_nodes:
                success, X, U, V = self.ACADO_connect(
                    self.graph.nodes[node_index].state,
                    self.graph.nodes[nn_node_index].state)
                # If successing while attempting to connect the two nodes
                # then add the new edge to the graph
                if success:
                    new_edges.append([(node_index, nn_node_index),
                                      Graph.new_path([X, U, V])])

                success, X, U, V = self.ACADO_connect(
                    self.graph.nodes[nn_node_index].state,
                    self.graph.nodes[node_index].state)
                # If successing while attempting to connect the two nodes
                # in reverse order then add the new edge to the graph
                if success:
                    new_edges.append([(nn_node_index, node_index),
                                      Graph.new_path([X, U, V])])

            if nb_best is not None:
                # TODO: explain to Guillermo why best = longest
                # best edges: the longest ones
                new_edges_states_array = np.array(
                    [edge[0] for edge in new_edges])
                new_edges_V_array = np.array([edge[1].V for edge in new_edges])

                best_edges_indexes = np.argpartition(
                    new_edges_V_array, self.nb_best)[:-(self.nb_best+1):-1]

                new_edges = [[new_edges_states_array[index],
                              new_edges[index][1]]
                             for index in best_edges_indexes]

        for new_edge in new_edges:
            self.graph.add_edge(new_edge[0], new_edge[1].X,
                                new_edge[1].U, new_edge[1].V)
        return len(new_edges)

    def improve_old(self, estimator, verbose=False):
        """Improve the prm using the approximators:
        - Replace some edges with betters paths
        - Tries to connect unconnected states
        """
        self.graph.edges.update(self.better_edges(nets, verbose=verbose))
        self.densify_random(nets, 20, verbose=verbose)
        self.connexify(nets, 5, verbose=verbose)

    def better_edges(self, nets, verbose=True):
        '''Return a ditc of edges that improve the PRM edge cost.'''
        EPS = 0.05

        edges_patch = {}
        for node0_index, node1_index in self.graph.edges:
            state0 = self.graph.nodes[node0_index].state
            state1 = self.graph.nodes[node1_index].state

            V_prm = self.graph.edges[(node0_index, node1_index)].V

            X_est, U_est, V_est = nets.trajectories(state0, state1)
            success, X, U, V = self.ACADO_connect(
                state0, state1, init=(X_est, U_est, V_est))

            if success:
                if (V < (1 - EPS) * V_prm):
                    if verbose:
                        print("Better connection: %d to %d (%.2f < %.2f)" % (
                            node0_index, node1_index, V, V_prm))

                    edges_patch[(node0_index, node1_index)
                                ] = Graph.new_path([X, U, V])

        return edges_patch

    def improve(self, estimator, verbose=True):
        """Improve the prm using the approximators:
        - Replace some edges with betters paths
        - Tries to connect unconnected states
        """
        EPS = 0.05

        better_edges = {}
        for node0_index, node1_index in self.graph.edges:
            state0 = self.graph.nodes[node0_index].state
            state1 = self.graph.nodes[node1_index].state

            V_prm = self.graph.edges[(node0_index, node1_index)].V

            X_est, U_est, V_est = estimator.trajectories(state0, state1)

            if V_est < (1 - EPS) * V_prm:
                success, X, U, V = self.ACADO_connect(
                    state0, state1, init=(X_est, U_est, V_est))
                if success and V < (1 - EPS) * V_prm:
                    if verbose:
                        print("Better connection: %d to %d (%.2f < %.2f)" % (
                            node0_index, node1_index, V, V_prm))
                    better_edges[(node0_index, node1_index)
                                 ] = Graph.new_path([X, U, V])

        self.graph.edges.update(better_edges)
        return not bool(better_edges)

    def densify_random(self, estimator=None, n=10, max_iter=10, verbose=False):
        """Try to connect more nodes using the NN approx.
        Randomly select n nodes in the graph and try to connect them
        to any other node of the graph.

        - estimator: networks estimating trajectories and value function
        - n: number of nodes to connect
        - max_iter: max number of connection attempt from each chose node
        """

        for i in range(n):
            # TODO: maybe something better than just random? Least connected?
            # use the V-value for a horizon limited vue?
            success = False
            areConnected = True
            iteration = 0

            # Selects two random nodes until it finds two unconnected nodes
            # or exceeds the max_iter iterations
            while areConnected and iteration < max_iter:
                iteration += 1
                nodes_indexes = random.choices(list(self.graph.nodes), k=2)
                areConnected = (tuple(nodes_indexes)
                                in self.graph.edges.keys()
                                or nodes_indexes[0] == nodes_indexes[1])

            if not areConnected:

                states = (self.graph.nodes[nodes_indexes[0]],
                          self.graph.nodes[nodes_indexes[1]])

                if verbose:
                    print('#%d: Connecting %d to %d' % (i,
                                                        nodes_indexes[0],
                                                        nodes_indexes[1]))

                # Tries to optimally connect the two nodes
                # TODO: Not done initially in the irepa code
                success, X_opt, U_opt, V_opt = self.opt_trajectories(
                    states, estimator,
                    verbose=True)
            # TODO: except ACADOerror

            # If an optimal path between the two nodes was found, then adds it
            # to the graph as a new edge
                if success:
                    self.graph.add_edge((nodes_indexes[0],
                                         nodes_indexes[1]),
                                        X_opt, U_opt, V_opt)
                    if verbose:
                        print('\t\t\t... Yes!')
                else:
                    if verbose:
                        print('\t\t\t... No!')

        return

    def densify_longer_traj(self, nb_attempt, min_path):
        """Picks two random states, finds their shortest path in the graph
        and then tries to directly connect them using the OCP warm-node1ed with
        the shortest path.
        - min_path: Minimum size of longer paths to consider
        """

        # TODO: Precise this min
        # Minimum size of longer paths to consider
        MIN_PATH_LEN = 3

        # TODO: new argument
        for _ in range(10):
            X_prm = []
            # TODO: random or function of "connectability"
            node1, node2 = random.choices(list(self.graph.nodes.keys()), k=2)
            if node1 in self.graph.nodes[node1].linked_to:
                continue
            shortest_path = self.graph.astar(node1, node2)
            if shortest_path is not None and len(shortest_path) >= min_path:
                state1 = self.graph.nodes[node1]
                state2 = self.graph.nodes[node2]
                for i in range(len(shortest_path)-1):
                    edge = shortest_path[i], shortest_path[i+1]
                    X_edge, _, _ = self.graph.edges[edge]
                    X_prm.append(X_edge)

                X_prm = np.vstack(X_prm)

                success, X, U, V = self.ACADO_connect(state1, state2, X_prm)
                if success:
                    self.graph.add_edge((node1, node2), X, U, V)

    def connexify(self, estimator, nb_connect=5, verbose=False):
        """
        For every pair of connex components in the graph, tries to connect
        nb_connect pairs of nodes.
        """
        connex_groups_id = list(self.graph.connex_groups)
        connex_pairs = permutations(connex_groups_id, 2)
        new_edges = []
        for conidx1, conidx2 in connex_pairs:
            for _ in range(nb_connect):
                node_idx1 = random.choice(self.graph.connex_groups[conidx1])
                node_idx2 = random.choice(self.graph.connex_groups[conidx2])
                state1 = self.graph.nodes[node_idx1]
                state2 = self.graph.nodes[node_idx2]
                success, X_opt, U_opt, V_opt = self.opt_trajectories(
                    (state1, state2), estimator,
                    verbose=verbose)
                if success:
                    new_edges.append(((node_idx1, node_idx2),
                                      X_opt, U_opt, V_opt))

        for edge in new_edges:
            self.graph.add_edge(*edge)

    def opt_trajectories(self, states, estimator=None, verbose=False):
        """Make an estimate of the trajectories to warm up the optimizer or
        just call the optimizer if no estimator"""
        if estimator:
            X_est, U_est, V_est = estimator.trajectories(*states)
            init_path = (X_est, U_est, V_est)
        else:
            init_path = None
        if verbose:
            print(f"\t\t Trying...")
        return self.ACADO_connect(states[0], states[1], init=init_path)


class Graph:

    save_fields = ('nodes', 'edges')

    def __init__(self, hdistance):

        # NamedTuple representing a path between two nodes
        # nodes = {node_id: state0}
        self.nodes = {}

        # edges = {(node_id,node_id1): Path(X, U, V)}
        self.edges = {}

        # connex_groups = {connex_group_id : [nodes_id]}
        self.connex_groups = {}

        # connex_elements = {nodes_id : [connex_group_id]}
        self.connex_elements = {}

        # ditance metric
        self.hdistance = hdistance

    def __str__(self):
        return f"""{len(self.nodes)} nodes, {len(self.edges)} edges \n
                Nodes: {self.nodes} \n
                Edges: {self.edges}"""

    def save(self, directory):
        """Save the graphs attributes to files in the directory"""
        for field in self.save_fields:
            np.save(pjoin(directory, field+'.npy'), self.__dict__[field])

    def load(self, directory):
        """Load the graphs attributes from files in the directory"""
        for field in self.save_fields:
            self.__dict__[field] = np.load(pjoin(directory, field)+'.npy')[()]

    def add_node(self, state, verbose=True):
        """
        Add a node to the graph with a defined state empty linked nodes
        Return index of note
        """
        node_index = len(self.nodes)
        # TODO: Might use a set instead -> better for lookup action
        self.nodes[node_index] = Node(state, [])

        # Creates a new connex group and adds to it the new node
        self.add_to_new_connex_group(node_index)
        if verbose:
            print(f"Added node [{node_index}:{state}] to graph")
            print(f"Node {node_index} is in connex element " +
                  f"{self.connex_elements[node_index]}\n")
        return node_index

    @staticmethod
    def new_path(path):
        """Return an Edge named tuple, path is [X,U,V] """
        return Path(path[0], path[1], path[2])

    def add_edge(self, nodes, X, U, V):
        """Add an edge to the graph. The nodes in the edge argument must
        already be in the graph.
        """
        assert(nodes[0] in self.nodes)
        assert(nodes[1] in self.nodes)

        if nodes[0] != nodes[1]:

            self.edges[nodes] = Graph.new_path([X, U, V])
            self.nodes[nodes[0]].linked_to.append(nodes[1])
            self.join_connex_groups(self.connex_elements[nodes[0]],
                                    self.connex_elements[nodes[1]])

    def add_to_new_connex_group(self, node_index):

        new_group_index = len(self.connex_groups)
        self.connex_groups[new_group_index] = [node_index]
        self.connex_elements[node_index] = new_group_index

    def join_connex_groups(self, connex_group_id0, connex_group_id1):

        if (connex_group_id0 != connex_group_id1):
            nodes_indexes_to_change = self.connex_groups.pop(connex_group_id1)
            self.connex_groups[connex_group_id0] += nodes_indexes_to_change

            changes = {
                node_index: connex_group_id0
                for node_index in nodes_indexes_to_change}
            self.connex_elements.update(changes)

    def node_list_to_state_list(self, node_list):
        """Given a list of node indices, return the list of associated states
        """
        return [self.nodes[node].state for node in node_list]

    # TODO finalize
    def nn_from_state(self, state, max_nn=1):
        """
        Return the max_nn nearest neighbours nodes indexes of state.
        """

        distances = np.array(
            [[node_index,
              self.hdistance(state, node.state)] for node_index, node
             in self.nodes.items() if node.state != state])

        if len(distances) <= max_nn:
            return list(self.nodes.keys())

        """ np.argpartition([hdistance],max_nn)[:max_nn] returns an unsorted
        list of node indices for the max_nn nearest neighbours nodes of state
        """
        nn_indexes = np.argpartition(distances[:, 1], max_nn)[:max_nn]

        # FIXME
        return distances[nn_indexes, 0]

    def get_path(self, node1, node2):
        # If nodes already connected, return edge path
        X_prm, U_prm, V_prm = [], [], 0
        if (node1, node2) in self.edges:
            return self.edges[(node1, node2)]
        shortest_path = self.astar(node1, node2)
        if shortest_path is None:
            return None
        print()
        print()
        print()
        print('SHORTEST PATH:', shortest_path)
        for i in range(len(shortest_path)-1):
            pair = shortest_path[i], shortest_path[i+1]
            X_edge, U_edge, V_edge = self.edges[pair]
            X_prm.append(X_edge)
            U_prm.append(U_edge)
            V_prm += V_edge

        X_prm = np.vstack(X_prm)
        U_prm = np.vstack(U_prm)

        return X_prm, U_prm, V_prm

    def astar(self, node1, node2):
        """
        Find the shortest between two nodes in the graph.
        node1 and node2 are both indices of nodes that should be in the graph.
        hdistance: heuristic distance used in the algorithm (euclidian, Value f)
        """
        if node1 not in self.nodes:
            raise ValueError('node ' + str(node1) + ' not in the graph')
        if node2 not in self.nodes:
            raise ValueError('node ' + str(node2) + ' not in the graph')

        # list of (node index, heuristic)
        frontier = PriorityQueue()
        frontier.put(node1, 0.)
        came_from = {}
        cost_so_far = {}
        came_from[node1] = None
        cost_so_far[node1] = 0.

        while not frontier.empty():
            # Current
            current = frontier.get()
            if current == node2:
                # path reconstitution node1ing from the end
                shortest_path = [node2]
                while shortest_path[-1] != node1:
                    shortest_path.append(came_from[shortest_path[-1]])

                return list(reversed(shortest_path))
            # search in nodes linked_to
            for neigh in self.nodes[current].linked_to:
                new_cost = cost_so_far[current] + \
                    self.edges[(current, neigh)].V
                # If newly visited node or already visited but with bigger cost
                if neigh not in cost_so_far or new_cost < cost_so_far[neigh]:
                    cost_so_far[neigh] = new_cost
                    dist_node2 = self.hdistance(self.nodes[current].state,
                                                self.nodes[node2].state)
                    anti_priority = new_cost + dist_node2
                    frontier.put(neigh, anti_priority)
                    came_from[neigh] = current

        # no path found
        return None


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def __str__(self):
        return str(self.elements)

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

import random
import numpy as np
from os.path import join as pjoin
import heapq

from collections import namedtuple


Path = namedtuple('Path', ['X', 'U', 'V'])


class PRM:
    """
    Simple generic PRM implementation using the Graph structure

    sample: generate a random state in the admissible state space
    connect: determines if 2 states are connectable and return the edges
             between the 2 nodes
    nb_sample: number of sample take from the state space
    nb_connect: number of nearest state to try to connect with
    nb_best: number of the best edges to keep
    """

    def __init__(self, sample, connect, nb_sample, nb_connect, nb_best=None):
        self.sample = sample
        self.ACADO_connect = connect
        self.nb_sample = nb_sample
        self.nb_connect = nb_connect
        self.nb_best = nb_best
        self.graph = Graph()

    def build_graph(self, hdistance):
        """Build the prm graph"""
        for _ in range(self.nb_sample):

            state = self.sample()
            node_index = self.graph.add_node(state)

        self.expand(n=self.nb_connect)

        """
            nearest_nodes = self.graph.nn_from_state(state, hdistance,
                                                     self.nb_connect)
            new_edges = []
            for nn_node_index in nearest_nodes:

                success, X, U, V = self.ACADO_connect(
                    self.graph.nodes[node_index],
                    self.graph.nodes[nn_node_index])
                ## If successing while attempting to connect the two nodes
                ## then add the new edge to the graph
                if success:
                    new_edges.append([(node_index, nn_node_index),
                                      Graph.new_path([X, U, V])])

                success, X, U, V = self.ACADO_connect(
                    self.graph.nodes[nn_node_index],
                    self.graph.nodes[node_index])
                ## If successing while attempting to connect the two nodes
                ## in reverse order then add the new edge to the graph
                if success:
                    new_edges.append([(nn_node_index, node_index),
                                      Graph.new_path([X, U, V])])

            if self.nb_best:
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

            return [self.graph.add_edge(
                new_edge[0],
                new_edge[1].X,
                new_edge[1].U,
                new_edge[1].V) for new_edge in new_edges]
        """

    def improve(self, nets):
        """Improve the prm using the approximators:
        - Replace some edges with betters paths
        - Tries to connect unconnected states
        """
        self.graph.edges.update(self.better_edges(nets))
        self.expand(nets, 20)

    def better_edges(self, nets, verbose=True):
        '''Return a ditc of edges that improve the PRM edge cost.'''
        EPS = 0.05

        edges_patch = {}
        for node0_index, node1_index in self.graph.edges:
            state0 = self.graph.nodes[node0_index]
            state1 = self.graph.nodes[node1_index]

            V_prm = self.graph.edges[node0_index, node1_index].V

            X_est, U_est, V_est = nets.trajectories(state0, state1)
            success, X, U, V = self.ACADO_connect(
                state0, state1, init=(X_est, U_est, V_est))

            if success:
                if V < (1 - EPS) * V_prm:
                    if verbose:
                        print("Better connection: %d to %d (%.2f vs %.2f)" % (
                            node0_index, node1_index, V, V_prm))
                    edges_patch[(node0_index, node1_index)
                                ] = Graph.new_path([X, U, V])

        return edges_patch

    def expand(self, estimator=None, n=10, verbose=True, max_iter=10):
        """Try to connect more nodes using the NN approx.
        - nets: approximators
        - n: number of trials"""
        # TODO: Not really an expansion since no new node is added to the graph

        for i in range(n):
            # TODO: maybe something better than just random? Least connected?
            success = False
            areConnected = True
            iteration = 0

            # TODO: use the V-value for a horizon limited vue
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
                if estimator:
                    X_est, U_est, V_est = estimator.connect(states)
                    init_path = (X_est, U_est, V_est)
                else:
                    init_path = None
                print(f"\t\t Trying...")
                success, X_opt, U_opt, V_opt = self.ACADO_connect(
                    states[0], states[1], init=init_path)
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

    # TODO
    def connexify(self):
        connex_groups_id = list(self.graph.connex_groups)
        if len(connex_groups_id) > 1:
            for id in connex_groups:
                if id in self.graph.connex_groups:
                    pass


class Graph:

    save_fields = ('nodes', 'edges')

    def __init__(self):

        # NamedTuple representing a path between two nodes
        # nodes = {node_id: state0}
        self.nodes = {}

        # edges = {(node_id,node_id1): Path(X, U, V)}
        self.edges = {}

        # connex_groups = {connex_group_id : [nodes_id]}
        self.connex_groups = {}

        # connex_elements = {nodes_id : [connex_group_id]}
        self.connex_elements = {}

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

    def add_node(self, state):
        """
        Add a node to the graph with a defined state and no linked nodes
        Return index of note
        """
        node_index = len(self.nodes)
        self.nodes[node_index] = state

        # Creates a new connex group and adds to it the new node
        self.add_to_new_connex_group(node_index)
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

    # FIXME
    def node_list_to_state_list(self, node_list):
        """Given a list of node indices, return the list of associated states
        """
        return [self.nodes[node][0] for node in node_list]

    # TODO finalize
    def nn_from_state(self, state, hdistance, max_nn=1):
        """
        Return the max_nn nearest neighbours nodes indexes of state.
        """

        distances = np.array(
            [[node_index,
              hdistance(state, node)] for node_index, node
             in self.nodes.items() if node != state])

        if len(distances) <= max_nn:
            return self.nodes.keys()

        """ np.argpartition([hdistance],max_nn)[:max_nn] returns an unsorted
        list of node indices for the max_nn nearest neighbours nodes of state
        """
        nn_indexes = np.argpartition(distances[:, 1], max_nn)[:max_nn]

        # FIXME
        return distances[nn_indexes, 0]

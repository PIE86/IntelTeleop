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
            nearest_nodes = self.graph.nn_from_state(state, hdistance,
                                                     self.nb_connect)
            new_edges = []
            for nn_node_index in nearest_nodes:

                success, X, U, V = self.ACADO_connect(self.graph.nodes[node_index],
                                                      self.graph.nodes[nn_node_index])
                """ If successing while attempting to connect the two nodes
                then add the new edge to the graph"""
                if success:
                    new_edges.append([(node_index, nn_node_index),
                                      Graph.new_path([X, U, V])])

                success, X, U, V = self.ACADO_connect(self.graph.nodes[nn_node_index],
                                                      self.graph.nodes[node_index])
                """ If successing while attempting to connect the two nodes
                in reverse order then add the new edge to the graph"""
                if success:
                    new_edges.append([(nn_node_index, node_index),
                                      Graph.new_path([X, U, V])])

            if self.nb_best:
                # TODO: explain to Guillermo why best = longest
                # best edges: the longest ones
                new_edges_states_array = np.array(
                    [edge[0] for edge in new_edges])
                new_edges_V_array = np.array([edge[1].V for edge in new_edges])

                best_edges_indexes = np.argpartition(new_edges_V_array,
                                                     self.nb_best)[:-(self.nb_best+1):-1]

                new_edges = [[new_edges_states_array[index], new_edges[index][1]]
                             for index in best_edges_indexes]

            return [self.graph.add_edge(new_edge[0],
                                        new_edge[1].X,
                                        new_edge[1].U,
                                        new_edge[1].V) for new_edge in new_edges]

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

    def expand(self, nets, n=10, verbose=True):
        """Try to connect more nodes using the NN approx.
        - nets: approximators
        - n: number of trials"""
        # TODO: Not really an expansion since no new node is added to the graph

        for i in range(n):
            # TODO: maybe something better than just random? Least connected?
            success = False
            areNotConnected = True

            # TODO: use the V-value for a horizon limited vue
            # Selects two random nodes until it finds two unconnected nodes
            while areNotConnected:
                node_index = random.choices(list(self.graph.nodes), k=2)
                areNotConnected = node_index not in self.graph.edges.keys()

            state = (self.graph.nodes[node_index[0]].state,
                     self.graph.nodes[node_index[1]].state)

            if verbose:
                print('#%d: Connecting %d to %d' % (i, i1, i2))

            # Tries to optimally connect the two nodes
            try:
                # TODO: Not done initially in the irepa code
                X, U, V = nets.connect(state)
                sucess, Xa, Ua, Va = self.ACADO_connect(s1, s2, init=(X, U, V))
            # TODO: except ACADOerror
            except:
                raise

            # If an optimal path between the two nodes was found, then adds it
            # to the graph as a new edge
            if success:
                self.graph.add_edge([i1, i2], [X, U, V])
                if verbose:
                    print('\t\t... Yes!')

        return


class Graph:

    save_fields = ('nodes', 'edges')

    def __init__(self):

        # NamedTuple representing a path between two nodes
        # nodes = {node_id: state0}
        self.nodes = {}

        # edges = {(node_id0,node_id1): Path(X, U, V)}
        self.edges = {}

        # connex_groups = {idx_connex_group : [idx_nodes]}
        self.connex_groups = {}

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

        self.edges[nodes] = Graph.new_path([X, U, V])

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

        distances = np.array([[node_index, hdistance(
            state, node)] for node_index, node in self.nodes.items() if node != state])

        if len(distances) <= max_nn:
            return self.nodes.keys()

        """ np.argpartition([hdistance],max_nn)[:max_nn] returns an unsorted 
        list of node indices for the max_nn nearest neighbours nodes of state """
        nn_indexes = np.argpartition(distances[:, 1], max_nn)[:max_nn]

        # FIXME
        return distances[nn_indexes, 0]

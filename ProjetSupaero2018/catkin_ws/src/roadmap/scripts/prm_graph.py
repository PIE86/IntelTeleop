import random
import numpy as np
from os.path import join as pjoin
import heapq

from collections import namedtuple

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
            node_new = self.sample()
            self.graph.add_node(node_new)
            nearest_nodes = self.graph.nearest_nodes(s, hdistance,
                                                     self.nb_connect)
            new_edges = []
            for close_node in nearest_nodes:
                # !! with Acado, to be done in the 2 ways !!
                close_s = self.graph.nodes[close_node][0]
                success, X, U, cost = self.connect(s, close_s)
                if success:
                    new_edges.append(Graph.new_stateEdge(
                        [node, close_node], [X, U, cost]))
                success, X, U, cost = self.connect(close_s, s)
                if success:
                    new_edges.append(Graph.new_stateEdge(
                        [clode_node, node], [X, U, cost]))

            if self.nb_best:
                # best edges: the longest ones
                new_edges = sorted(new_edges, lambda e: e[1][2])[:self.nb_best]
            for edge in new_edges:
                self.graph.add_edge(edge)

    def improve(self, nets):
        """Improve the prm using the approximators:
        - Replace some edges with betters paths
        - Tries to connect unconnected states
        """
        self.graph.edges.update(self.better_edges(nets))
        self.expand(nets, 20)

    def better_edges(self, nets, verbose=True):
        '''Return a patch that improve the PRM edge cost.'''
        EPS = 0.05

        edges_patch = {}
        for i1, i2 in self.graph.edges:
            s1 = self.graph.nodes[i1][0]
            s2 = self.graph.nodes[i2][0]
            Tp = self.graph.edges[i1, i2][2]
            try:
                X, U, T = nets.trajectories(s1, s2)
                Xa, Ua, Ta = self.connect(s1, s2, init=(X, U, T))
            # TODO: No connection available Exception
            except Exception:
                continue
            if Ta < (1 - EPS) * Tp:
                if verbose:
                    print("Better connection: %d to %d (%.2f vs %.2f)" % (
                        i1, i2, Ta, Tp))
                edges_patch[(i1, i2)] = (Xa, Ua, Ta)
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
            while areNotConnected:
                node_index = random.choices(list(self.graph.nodes), k=2)
                areNotConnected = node_index not in self.graph.edges.keys()

            state = self.graph.nodes[node_index[0]
                                     ].state, self.graph.nodes[node_index[1]].state

            if verbose:
                print('#%d: Connecting %d to %d' % (i, i1, i2))
            try:
                # TODO: Not done initially in the irepa code
                X, U, T = nets.connect(state)

                sucess, Xa, Ua, Ta = self.ACADO_connect(s1, s2, init=(X, U, T))

            # TODO: No connection available Exception
            if success:
                new_edge = self.graph.new_edge([i1, i2], [X, U, T])
                self.graph.add_edge([i1, i2], [X, U, T])
                if verbose:
                    print('\t\t... Yes!')


class Graph:
    save_fields = ('nodes', 'edges')

    def __init__(self):

        # NamedTuple representing a path between two nodes
        self.Path = namedtuple('Path', ['X', 'U', 'V'])

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
        """Add a node to the graph with a defined state and no linked nodes"""
        self.nodes[len(self.nodes)] = state

    def new_path(self, edge):
        """Return an Edge named tuple, edge is [X,U,V] """
        return self.Path(edge[0], edge[1], edge[2])

    def add_edge(self, nodes, X, U, V):
        """Add an edge to the graph. The nodes in the edge argument must
        already be in the graph.
        """

        assert(nodes[0] in self.nodes)
        assert(nodes[1] in self.nodes)

        self.edges[nodes] = new_path(X,U,V)

    # FIXME
    def node_list_to_state_list(self, node_list):
        """Given a list of node indices, return the list of associated states
        """
        return [self.nodes[node][0] for node in node_list]

    # TODO finalize
    def nn_from_state(self, state, hdistance, max_nn=1):
        """
        Return at max the nb_connect nodes close to the state.
        """

        distances = [[node_index, hdistance(
            state, node)] for node_index, node in self.nodes.item() if node != state]

        # TODO delete
        max_nn = min(max_nn, len(distances))
        # argpartition: return a list of node indices, the n first of which
        # correspond to the n lowest distance unsorted
        if len(distances) <= max_nn:
            return existing_nodes

        # FIXME
        return np.argpartition(distances, max_nn)[:max_nn]

import os
import heapq
import numpy as np
from itertools import permutations
from os.path import join as pjoin
from collections import namedtuple

from networks import resample

# Resampled shortest path trajectories size
MAXTRAJLENGTH = 40
# Directory where the PRM should be stored (in ~/.ros/)
DATADIR = 'irepa_data'

Path = namedtuple('Path', ['X', 'U', 'V'])
Node = namedtuple('Node', ['state', 'linked_to'])


class PRM:
    """
    Simple generic PRM implementation using the Graph structure.

    sample: generate a random state in the admissible state space
    connect: determines if 2 states are connectable and return the edge
             trajectories between the 2 nodes
    graph: Graph data structure containing nodes and edges of the PRM
    visibility_horizon: value representing the maximum "length" of the
                        trajectory connectable. Not used properly at the moment
                        because of unanswered questions: what measure should
                        be used (value function? euclidian distance?),
                        how to fix the horizon value?
    """

    def __init__(self, sample_fun, connect_fun, hdistance):
        self.sample = sample_fun
        self.ACADO_connect = connect_fun
        self.graph = Graph(hdistance)
        self.visibility_horizon = 10  # arbitrary value

        if not os.path.exists(DATADIR):
            os.makedirs(DATADIR)

    def add_nodes(self, nb_sample=40, verbose=True):
        """Add a given number of nodes sampled using sampling client"""
        samples = self.sample(nb_sample)
        for state in samples:
            self.graph.add_node(state)

    def expand(self, estimator, first=False):
        """
        Expand the PRM by trying to connect all unconnected nodes.

        For each pair of unconnected nodes
        if distance > visibility horizon: # equivalent to longer_traj
          p* <- shortest path in PRM
          E <- ACADO(init = p*)
        else: # equivalent to densify knn
          E <- ACADO(init = 0 or estimator)

        :param estimator: trajectories estimator built with IREPA
        :param first: if true, use astar instead of the estimator to init
                      the optimal control solver
        :type estimator: networks.Networks
        :type first: bool
        """
        # for monitoring purposes: compare astar and estimator inits
        nb_astar, nb_est, nb_attempt = 0, 0, 0

        for (node1, node2), distance in zip(*self.unconnected_2_nn()):
            nb_attempt += 1

            # TODO: figure out this visibility horizon
            # Maybe try the first iteration with a low MAX_ITERATIONS
            # WITHOUT the visibility_horizon to have a heuristic?
            # arbitrary euclidian distance value seems to be too... arbitrary
            if first:
                path_astar = None
                if distance > self.visibility_horizon:
                    print(node1, node2, 'too far')
                    path_astar = self.graph.get_path(node1, node2)

                success, X, U, V = self.ACADO_connect(
                    self.graph.nodes[node1].state,
                    self.graph.nodes[node2].state,
                    init=path_astar)
                nb_astar += success

            else:
                s1 = self.graph.nodes[node1].state
                s2 = self.graph.nodes[node2].state
                path_est = estimator.trajectories(s1, s2)
                success, X, U, V = self.ACADO_connect(
                    self.graph.nodes[node1].state,
                    self.graph.nodes[node2].state,
                    init=path_est)
                nb_est += success

            # If successing while attempting to connect the two nodes
            # then add the new edge to the graph
            if success:
                self.graph.add_edge((node1, node2), X, U, V)

        return nb_astar, nb_est, nb_attempt

    def unconnected_2_nn(self):
        """
        Return a list of unconnected node pairs in the Graph as well as the
        heuristic distances between nodes. This distance list is used
        unproperly as explained in class doc.
        """
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
        """Return true if the PRM graph is fully connected"""
        nb_nodes, nb_edges = len(self.graph.nodes), len(self.graph.edges)
        return nb_edges == nb_nodes*(nb_nodes-1)

    def improve(self, estimator, verbose=True):
        """
        Improve the prm using the approximators:
            - Replace some edges with betters paths
            - Tries to connect unconnected states
        :param estimator: trajectories estimator built with IREPA
        :param verbose: True if logs are needed
        :type estimator: networks.Networks
        :type verbose: bool
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
                        print("""    ---------> BETTER connection:
                        %d to %d (%.2f < %.2f)""" % (
                            node0_index, node1_index, V, V_prm))
                    better_edges[(node0_index, node1_index)
                                 ] = Graph.new_path([X, U, V])

        self.graph.edges.update(better_edges)
        return not bool(better_edges)


class Graph:

    """
    Data structure representing a graph to be built by the PRM.
    """

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
        # not used in final version but useful to check connexity
        self.connex_elements = {}

        # ditance metric
        self.hdistance = hdistance

    def __str__(self):
        return """{} nodes, {} edges \n
                Nodes: {self.nodes} \n
                Edges: {self.edges}""".format(len(self.nodes), len(self.edges))

    def np_save(self, directory):
        """Save the graphs attributes to files in the directory"""
        for field in self.save_fields:
            np.save(pjoin(directory, field+'.npy'), self.__dict__[field])

    def np_load(self, directory):
        """Load the graphs attributes from files in the directory"""
        for field in self.save_fields:
            self.__dict__[field] = np.load(pjoin(directory, field)+'.npy')[()]

    def add_node(self, state, verbose=True):
        """
        Add a node to the graph with a defined state and empty linked nodes.
        Return node index.

        :param state: state in the admissible state space associated with the
                      node the new node
        """
        node_index = len(self.nodes)
        self.nodes[node_index] = Node(state, [])

        # Creates a new connex group and adds to it the new node
        self.add_to_new_connex_group(node_index)
        if verbose:
            print("Added node [{}:{}] to graph".format(node_index, state))
            print("Node {} is in connex element ".format(node_index) +
                  "{}\n".format(self.connex_elements[node_index]))
        return node_index

    @staticmethod
    def new_path(path):
        """Return an Edge named tuple, path is [X,U,V] """
        return Path(path[0], path[1], path[2])

    def add_edge(self, nodes, X, U, V):
        """
        Add an edge to the graph. The nodes in the edge argument must
        already be in the graph.

        :param nodes: nodes indices pair
        :param X: states trajectory
        :param U: controls trajectory
        :param V: trajectory cost
        :type nodes: tuple of 2 inits
        :type X: numpy.array
        :type U: numpy.array
        :type V: float
        """
        assert(nodes[0] in self.nodes)
        assert(nodes[1] in self.nodes)

        if nodes[0] != nodes[1]:
            self.edges[nodes] = Graph.new_path([X, U, V])
            self.nodes[nodes[0]].linked_to.append(nodes[1])
            self.join_connex_groups(self.connex_elements[nodes[0]],
                                    self.connex_elements[nodes[1]])

    def add_to_new_connex_group(self, node_index):
        """
        Add a new connex group associated to a node.
        """
        new_group_index = len(self.connex_groups)
        self.connex_groups[new_group_index] = [node_index]
        self.connex_elements[node_index] = new_group_index

    def join_connex_groups(self, connex_group_id0, connex_group_id1):
        """Merge nodes from 2 connex groups"""
        if connex_group_id0 != connex_group_id1:
            nodes_indexes_to_change = self.connex_groups.pop(connex_group_id1)
            self.connex_groups[connex_group_id0] += nodes_indexes_to_change

            changes = {
                node_index: connex_group_id0
                for node_index in nodes_indexes_to_change}
            self.connex_elements.update(changes)

    def node_list_to_state_list(self, node_list):
        return [self.nodes[node].state for node in node_list]

    def get_path(self, node1, node2):
        """
        Find the oriented shorted path between 2 nodes already present in the
        graph if it exists and return the aggregated and resample states and
        control trajectories.

        :param node1: node index from which the path is computed
        :param node2: node index to which the path is computed
        :type node1: int
        :type node2: int
        """
        # If nodes already connected, return edge path
        X_prm, U_prm, V_prm = [], [], 0
        if (node1, node2) in self.edges:
            return self.edges[(node1, node2)]
        shortest_path = self.astar(node1, node2)
        if shortest_path is None:
            return None

        print('\nSHORTEST PATH:', shortest_path)
        for i in range(len(shortest_path)-1):
            pair = shortest_path[i], shortest_path[i+1]
            X_edge, U_edge, V_edge = self.edges[pair]
            X_prm.append(X_edge)
            U_prm.append(U_edge)
            V_prm += V_edge

        X_prm = resample(np.vstack(X_prm), MAXTRAJLENGTH)
        U_prm = resample(np.vstack(U_prm), MAXTRAJLENGTH)

        return X_prm, U_prm, V_prm

    def astar(self, node1, node2):
        """
        Return the shortest between two nodes present in the graph if
        it exists.

        :param node1: node index from which the path is computed
        :param node2: node index to which the path is computed
        :param hdistance: heuristic distance used in the algorithm (e.g.
                          euclidian, Value f)
        :type node1: int
        :type node2: int
        :type hdistance: function
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

    def total_cost(self):
        """Return the sum of all edges costs"""
        return sum(self.edges[e].V for e in self.edges)


class PriorityQueue:
    """Implementation of a priority queue based on the heapq module"""

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

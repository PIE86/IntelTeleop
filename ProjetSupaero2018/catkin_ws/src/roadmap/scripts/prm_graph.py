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

    def __init__(self, sample, connect):
        self.sample = sample
        self.ACADO_connect = connect
        self.graph = Graph()

    def add_nodes(self, nb_sample=40):
        """Add a given number of nodes"""
        # TODO: add an option to "guide this adding"
        # Ex: add nodes between 2 unconnectable nodes...
        for _ in range(nb_sample):

            state = self.sample()
            node_index = self.graph.add_node(state)

    def densify_knn(self, hdistance, nb_connect=3, nb_best=None):
        """Build the prm graph
        nb_sample: number of sample take from the state space
        nb_connect: number of nearest state to try to connect with
        nb_best: number of the best edges to keep
        """
        for node_index, node in self.graph.nodes.items():
            nearest_nodes = self.graph.nn_from_state(node.state, hdistance,
                                                     nb_connect)
            new_edges = []
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

            return [self.graph.add_edge(
                new_edge[0],
                new_edge[1].X,
                new_edge[1].U,
                new_edge[1].V) for new_edge in new_edges]

    def improve(self, nets):
        """Improve the prm using the approximators:
        - Replace some edges with betters paths
        - Tries to connect unconnected states
        """
        self.graph.edges.update(self.better_edges(nets))
        # TODO: densify_random can be viewed as a Densify? --> YES! Desnify random
        self.densify_random(nets, 20)
        self.connexify(nets, 5)

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
                if V < (1 - EPS) * V_prm:
                    if verbose:
                        print("Better connection: %d to %d (%.2f vs %.2f)" % (
                            node0_index, node1_index, V, V_prm))
                    edges_patch[(node0_index, node1_index)
                                ] = Graph.new_path([X, U, V])

        return edges_patch

    def densify_random(self, estimator=None, n=10, max_iter=10, verbose=True):
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

                states = (self.graph.nodes[nodes_indexes[0]].state,
                          self.graph.nodes[nodes_indexes[1]].state)

                if verbose:
                    print('#%d: Connecting %d to %d' % (i,
                                                        nodes_indexes[0],
                                                        nodes_indexes[1]))

                # Tries to optimally connect the two nodes
                # TODO: Not done initially in the irepa code
                success, X_opt, U_opt, V_opt = self.opt_trajectories(states,
                                                                     estimator)
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

    def densify_longer_traj(self, hdistance):
        """Pick two random states, find their shortest path in the graph
        then try to directly connect them using the OCP warm-started with
        the shortest path."""

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
            shortest_path = astar(node1, node2, self.graph, hdistance)
            if shortest_path is not None and len(shortest_path) >= MIN_PATH_LEN:
                state1 = self.graph.nodes[node1].state
                state2 = self.graph.nodes[node2].state
                for i in range(len(shortest_path)-1):
                    edge = shortest_path[i], shortest_path[i+1]
                    X_edge, _, _ = self.graph.edges[edge]
                    X_prm.append(X_edge)

                X_prm = np.vstack(X_prm)

                success, X, U, V = self.ACADO_connect(state1, state2, X_prm)
                if success:
                    self.graph.add_edge((node1, node2), X, U, V)

    def connexify(self, estimator, nb_connect=5):
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
                state1 = self.graph.nodes[node_idx1].state
                state2 = self.graph.nodes[node_idx2].state
                success, X_opt, U_opt, V_opt = self.opt_trajectories(
                                               (state1, state2), estimator)
                if success:
                    new_edges.append(((node_idx1, node_idx2),
                                      X_opt, U_opt, V_opt))

        for edge in new_edges:
            self.graph.add_edge(*edge)

    def opt_trajectories(self, states, estimator=None):
        """Make an estimate of the trajectories to warm up the optimizer or
        just call the optimizer if no estimator"""
        if estimator:
            X_est, U_est, V_est = estimator.trajectories(*states)
            init_path = (X_est, U_est, V_est)
        else:
            init_path = None
        print(f"\t\t Trying...")
        return self.ACADO_connect(states[0], states[1], init=init_path)


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
        Add a node to the graph with a defined state empty linked nodes
        Return index of note
        """
        node_index = len(self.nodes)
        # TODO: Might use a set instead -> better for lookup action
        self.nodes[node_index] = Node(state, [])

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
    def nn_from_state(self, state, hdistance, max_nn=1):
        """
        Return the max_nn nearest neighbours nodes indexes of state.
        """

        distances = np.array(
            [[node_index,
              hdistance(state, node.state)] for node_index, node
             in self.nodes.items() if node.state != state])

        if len(distances) <= max_nn:
            return list(self.nodes.keys())

        """ np.argpartition([hdistance],max_nn)[:max_nn] returns an unsorted
        list of node indices for the max_nn nearest neighbours nodes of state
        """
        nn_indexes = np.argpartition(distances[:, 1], max_nn)[:max_nn]

        # FIXME
        return distances[nn_indexes, 0]


def astar(start, goal, graph, hdistance):
    """
    Find the shortest between two nodes in the graph.
    start and goal are both indices of nodes that should be in the graph.
    hdistance: heuristic distance used in the algorithm (euclidian, Value f)
    """
    if start not in graph.nodes:
        raise ValueError('node ' + str(start) + ' not in the graph')
    if goal not in graph.nodes:
        raise ValueError('node ' + str(goal) + ' not in the graph')

    # list of (node index, heuristic)
    frontier = PriorityQueue()
    frontier.put(start, 0.)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0.

    while not frontier.empty():
        # Current
        current = frontier.get()
        if current == goal:
            # path reconstitution starting from the end
            shortest_path = [goal]
            while shortest_path[-1] != start:
                shortest_path.append(came_from[shortest_path[-1]])

            return list(reversed(shortest_path))
        # search in nodes linked_to
        for neigh in graph.nodes[current].linked_to:
            new_cost = cost_so_far[current] + graph.edges[(current, neigh)].V
            # If newly visited node or already visited but with bigger cost
            if neigh not in cost_so_far or new_cost < cost_so_far[neigh]:
                cost_so_far[neigh] = new_cost
                dist_goal = hdistance(graph.nodes[current].state,
                                      graph.nodes[goal].state)
                anti_priority = new_cost + dist_goal
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

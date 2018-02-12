import random
import numpy as np
from os.path import join as pjoin
import heapq


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
        self.connect = connect
        self.nb_sample = nb_sample
        self.nb_connect = nb_connect
        self.nb_best = nb_best
        self.graph = Graph()

    def build_graph(self, hdistance):
        """Build the prm graph"""
        for _ in range(self.nb_sample):
            node = self.graph.new_node()
            s = self.sample()
            self.graph.add_node(node, s)
            nearest_nodes = self.graph.nearest_nodes(s, hdistance,
                                                     self.nb_connect)
            new_edges = []
            for close_node in nearest_nodes:
                # !! with Acado, to be done in the 2 ways !!
                s = self.graph.nodes[node][0]
                close_s = self.graph.nodes[close_node][0]
                success, X, U, cost = self.connect(s, close_s)
                if success:
                    new_edges.append(((node, close_node), (X, U, cost)))
                success, X, U, cost = self.connect(close_s, s)
                if success:
                    new_edges.append(((close_node, node), (X, U, cost)))

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
            i1, i2 = random.choice(list(self.graph.edges.keys()))
            s1, s2 = self.graph.nodes[i1][0], self.graph.nodes[i2][0]
            if i2 not in self.graph.nodes[i1][1]:
                if verbose:
                    print '#%d: Connecting %d to %d' % (i, i1, i2)
                try:
                    # TODO: Not done initially in the irepa code
                    X, U, T = nets.trajectories(s1, s2)
                    sucess, Xa, Ua, Ta = self.connect(s1, s2, init=(X, U, T))
                # TODO: No connection available Exception
                except Exception:
                    success = False
                if success:
                    new_edge = ((i1, i2), (X, U, T))
                    self.graph.add_edge(new_edge)
                    if verbose:
                        print('\t\t... Yes!')


class Graph:
    save_fields = ('nodes', 'edges')

    def __init__(self):
        # idx_node: (state, linked_to, linked_from)
        # 4: ((0.56, 4.2), [4, 12, 5], [4, 12])
        self.nodes = {}
        # (idx_node1, idx_node2): (state_trajectory, cmd_trajectory, cost)
        # (i1, i2): (X, U, T)
        # (4, 2): ([(1, 2), (1, 2), (1, 2)], [(1, 2), (1, 2), (1, 2)], 12)
        self.edges = {}

    def __str__(self):
        desc = [
            str(len(self.nodes)) + ' nodes',
            str(len(self.edges)) + ' edges',
            'Nodes: ' + str(self.nodes) + '\n',
            'Edges: ' + str(self.edges)
        ]
        return '\n'.join(desc)

    def save(self, directory):
        """Save the graphs attributes to files in the directory"""
        for field in self.save_fields:
            np.save(pjoin(directory, field+'.npy'), self.__dict__[field])

    def load(self, directory):
        """Load the graphs attributes from files in the directory"""
        for field in self.save_fields:
            self.__dict__[field] = np.load(pjoin(directory, field)+'.npy')[()]

    def new_node(self):
        """Return new node index"""
        return len(self.nodes)

    def add_node(self, idx, state):
        """Add a node to the graph with a defined state and no linked nodes"""
        self.nodes[idx] = (state, [], [])

    def add_edge(self, e):
        """Add an edge to the graph. The nodes in the edge argument must
        already be in the graph.
        edge -> ((i1, i2), (X, U, T))"""
        self.nodes[e[0][0]][1].append(e[0][1])
        self.nodes[e[0][1]][2].append(e[0][0])
        self.edges[e[0]] = e[1]

    def node_list_to_state_list(self, node_list):
        """Given a list of node indices, return the list of associated states
        """
        return [self.nodes[node][0] for node in node_list]

    def nearest_nodes(self, state, hdistance, nb_connect, new=True):
        """
        Return at max the nb_connect nodes close to the state.
        new bool: enables to avoid comparing the new node with itself if True
        """
        # -1 to avoid comparing the new node with itself but dirty
        existing_nodes = list(range(len(self.nodes) - 1*new))
        distances = [hdistance(state, self.nodes[node_g][0])
                     for node_g in existing_nodes]
        nb_connect = min(nb_connect, len(distances))
        # argpartition: return a list of node indices, the n first of which
        # correspond to the n lowest distance unsorted
        if len(distances) <= nb_connect:
            return existing_nodes

        return np.argpartition(distances, nb_connect)[:nb_connect]


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
        for neigh in graph.nodes[current][1]:
            new_cost = cost_so_far[current] + graph.edges[(current, neigh)]
            # If newly visited node or already visited but with bigger cost
            if neigh not in cost_so_far or new_cost < cost_so_far[neigh]:
                cost_so_far[neigh] = new_cost
                dist_goal = hdistance(graph.nodes[current][0],
                                      graph.nodes[goal][0])
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

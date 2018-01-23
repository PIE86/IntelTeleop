from os.path import join as pjoin
import random
import numpy as np
import heapq


class Graph:
    save_fields = ('nodes', 'edges')

    def __init__(self, state_space, hdistance):
        self.state_space = state_space
        self.hdistance = hdistance

        # idx_node: (state, linked_to, linked_from)
        # ex: 4: ((0.56, 4.2), [4, 12, 5], [4, 12])
        self.nodes = {}
        # (idx_node1, idx_node2): distance
        # ex: (4, 2): 2.5
        self.edges = {}

    def __str__(self):
        desc = [
            str(len(self.nodes)) + ' nodes',
            str(len(self.edges)) + ' edges',
            'Nodes: ' + str(self.nodes) + '\n',
            'Edges: ' + str(self.edges)
        ]
        return '\n'.join(desc)

    def save(self, path):
        for field in self.save_fields:
            np.save(pjoin(path, field+'.npy'), self.__dict__[field])

    def load(self, path):
        for field in self.save_fields:
            self.__dict__[field] = np.load(pjoin(path, field)+'.npy')[()]

    def new_node(self):
        return len(self.nodes), self.state_space.rdm_state()

    def add_node(self, idx, state):
        self.nodes[idx] = (state, [], [])

    def add_edge(self, edge):
        self.nodes[edge[0]][1].append(edge[1])
        self.nodes[edge[1]][2].append(edge[0])
        self.edges[(edge[0], edge[1])] = edge[2]

    def node_list_to_state_list(self, node_list):
        return [self.nodes[node][0] for node in node_list]

    def closest_nodes(self, state, nb_connect, new=True):
        """
        new bool: enables to avoid comparing the new node with itself if True
        """
        # -1 to avoid comparing the new node with itself but dirty
        existing_nodes = list(range(len(self.nodes) - 1*new))
        distances = [self.hdistance(state, self.nodes[node_g][0])
                     for node_g in existing_nodes]
        nb_connect = min(nb_connect, len(distances))
        # argpartition: return a list of node indices, the n first of which
        # correspond to the n lowest distance unsorted
        if len(distances) <= nb_connect:
            return existing_nodes

        return np.argpartition(distances, nb_connect)[:nb_connect]


class StateSpace:
    def __init__(self, bounds, seed=None):
        self.bounds = bounds
        random.seed(seed)

    def rdm_state(self):

        return [b[0] + random.random()*(b[1] - b[0]) for b in self.bounds]


def astar(start, goal, graph, hdistance):
    """Find the shortest between two nodes in the graph"""
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


def euclid(s1, s2):
    return np.sqrt((s2[0]-s1[0])**2 + (s2[1]-s1[1])**2)

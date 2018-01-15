import random
import numpy as np
import heapq

import rospy
from obstacles.srv import *

CHECKPOINT_SERVICE = 'check_point'

# node: index of a node
# state: tuple representing the state associated or not with a node

def main():
    rospy.init_node('init_the_prm')
    rospy.wait_for_service(CHECKPOINT_SERVICE)

    state_space = StateSpace([(0, 10), (0, 5)])
    nb_sample = 10
    nb_connect = 3
    graph = prm_init(state_space, nb_sample, nb_connect)
    print(graph)


def prm_init(state_space, nb_sample, nb_connect, nb_best=None):
    """
    nb_sample: number of sample take from the state space
    nb_connect: number of nearest state to try to connect with
    nb_best: number of the best edges to keep
    """

    graph = Graph(state_space, euclid)
    for _ in range(nb_sample):
        node, s = graph.new_node()
        if valid_state(s):
            graph.add_node(node, s)
        else:
            continue

        closest_nodes = graph.closest_nodes(s, nb_connect)
        new_edges = []
        for close_node in closest_nodes:
            # !! with Acado, to be done in the 2 ways !!
            conn = connection(node, close_node)
            if conn[0]:
                new_edges.append((node, close_node, conn[1]))
            conn = connection(close_node, node)
            if conn[0]:
                new_edges.append((close_node, node, conn[1]))

        if nb_best:
            # best edges: the longest ones
            new_edges = sorted(new_edges, key=lambda edge: edge[2])[:nb_best]
        for edge in new_edges:
            graph.add_edge(edge)

    return graph


def valid_state(s):
    try:
        get_if_valid = rospy.ServiceProxy(SERVICE_NAME, CheckPoint)
        resp = get_if_valid(*s)
        return resp.is_valid
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    # return s[0] + s[1] < 8 or s[0] + s[1] > 10


def connection(node1, node2):
    # dumb function, return a false
    return (True, random.random())


class StateSpace:
    def __init__(self, bounds, seed=None):
        self.bounds = bounds
        random.seed(seed)

    def rdm_state(self):

        return [b[0] + random.random()*(b[1] - b[0]) for b in self.bounds]


class Graph:
    def __init__(self, state_space, hdistance):
        self.state_space = state_space
        self.hdistance = hdistance

        # idx_node: (state, linked_nodes)
        # ex: 4: ((0.56, 4.2), [4, 12, 5])
        self.nodes = {}
        # (idx_node1, idx_node2): distance
        # ex: (4, 2): 2.5
        self.edges = {}

    def __str__(self):
        return 'Nodes: ' + str(self.nodes) + '\n\n' + 'Edges: ' + str(self.edges)

    def save(self, path):
        pass

    def load(self, path):
        pass

    def new_node(self):
        return len(self.nodes), self.state_space.rdm_state()

    def add_node(self, idx, state):
        self.nodes[idx] = (state, [])

    def add_edge(self, edge):
        self.nodes[edge[0]][1].append(edge[1])
        self.nodes[edge[1]][1].append(edge[0])
        self.edges[(edge[0], edge[1])] = edge[2]

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


def astar(start, goal, graph, hdistance):
    """Find the shortest between two nodes in the graph"""
    if start not in graph.nodes:
        raise ValueError('node ' + str(start) + ' not in the graph')
    if goal not in graph.nodes:
        raise ValueError('node ' + str(goal) + ' not in the graph')

    # list of (node index, heuristic)
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()
        print('came_from')
        print(came_from)
        print('cost_so_far')
        print(cost_so_far)
        if current == goal:
            # path reconstitution starting from the end
            shortest_path = [goal]
            while shortest_path[-1] != start:
                shortest_path.append(came_from[shortest_path[-1]])

            return list(reversed(shortest_path))
        for neigh in graph.nodes[current][1]:

            new_cost = cost_so_far[current] + graph.edges[(current, neigh)]

            if neigh not in cost_so_far or new_cost < cost_so_far[neigh]:
                cost_so_far[neigh] = new_cost
                dist_goal = hdistance(graph.nodes[current][0], graph.nodes[goal][0])
                priority = new_cost + dist_goal
                frontier.put(neigh, priority)
                came_from[neigh] = current

    # no path found
    return None


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

def euclid(s1, s2):
    return np.sqrt((s2[0]-s1[0])**2 + (s2[1]-s1[1])**2)

if __name__ == '__main__':
    state_space = StateSpace([(0, 10), (0, 5)])
    nb_sample = 10
    nb_connect = 3
    graph = prm_init(state_space, nb_sample, nb_connect)
    print(graph)

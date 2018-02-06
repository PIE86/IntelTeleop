#!/usr/bin/env python

"""A path_finding service used in jalon 1 to generate trajectories.
Retrieves information from data files to build the prm graph and then
use astar on it."""

import os
import rospy
import rospkg
from roadmap.srv import PathFinding, PathFindingResponse
from geometry_msgs.msg import Point

import prm

PACKAGE_NAME = 'roadmap'
ros_package = rospkg.RosPack()
GRAPH_DATA_PATH = os.path.join(ros_package.get_path(PACKAGE_NAME), 'data')

X = (0, 10)
Y = (0, 5)
state_space = prm.StateSpace([X, Y])

graph = prm.Graph(state_space, prm.euclid)
graph.load(GRAPH_DATA_PATH)

NB_CONNECT = 10


def callback(start_end):
    p1 = start_end.p1
    p2 = start_end.p2
    s1 = p1.x, p1.y
    s2 = p2.x, p2.y
    print('s1, s2:', s1, s2)
    return find_path(s1, s2)


def find_path(s1, s2):
    """
    Given 2 states, compute the shortest path from the 2 closest nodes  in the
    graph using astar and return the complete path.
    """
    n1 = graph.closest_nodes(s1, NB_CONNECT)[0]
    n2 = graph.closest_nodes(s2, NB_CONNECT)[0]
    node_path = prm.astar(n1, n2, graph, prm.euclid)
    print('node_path:', node_path)
    state_path = [s1] + graph.node_list_to_state_list(node_path) + [s2]
    point_path = [Point(s[0], s[1], 0.) for s in state_path]

    return PathFindingResponse(point_path)


def path_finding_service():
    rospy.init_node('path_finding')
    rospy.Service('find_path', PathFinding, callback)
    print "Ready to find paths"
    rospy.spin()


if __name__ == '__main__':
    path_finding_service()

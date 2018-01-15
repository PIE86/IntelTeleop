#!/usr/bin/env python
import os
import random

try:
    import rospy
    import rospkg
    from pie86_obstacles.srv import *
except:
    print('ROS not runnning')

import prm

CHECKPOINT_SERVICE = 'check_point'
PACKAGE_NAME = 'roadmap'

rospackage = rospkg.RosPack()
PACKAGE_DATA_PATH = os.path.join(rospackage.get_path(PACKAGE_NAME), 'data')
if not os.path.exists(PACKAGE_DATA_PATH):
    os.makedirs(PACKAGE_DATA_PATH)

NB_SAMPLE = 100
NB_CONECT = 2
# node: index of a node
# state: tuple representing the state associated or not with a node

def main():
    rospy.init_node('init_the_prm')
    rospy.loginfo('Init PRM initialized')
    rospy.wait_for_service('check_point')
    rospy.loginfo('End of wait for check_point')

    state_space = prm.StateSpace([(0, 10), (0, 5)], seed=0)
    graph = prm_init(state_space, NB_SAMPLE, NB_CONECT)
    # rospy.loginfo(graph)
    graph.save(os.path.join(PACKAGE_DATA_PATH))


def prm_init(state_space, nb_sample, nb_connect, nb_best=None):
    """
    nb_sample: number of sample take from the state space
    nb_connect: number of nearest state to try to connect with
    nb_best: number of the best edges to keep
    """

    graph = prm.Graph(state_space, prm.euclid)
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
            s = graph.nodes[node][0]
            close_s = graph.nodes[close_node][0]
            conn = connection(s, close_s)
            if conn[0]:
                new_edges.append((node, close_node, conn[1]))
            conn = connection(close_s, s)
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
        get_if_valid = rospy.ServiceProxy(CHECKPOINT_SERVICE, CheckPoint)
        resp = get_if_valid(*s)
        return resp.is_valid
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    # return s[0] + s[1] < 8 or s[0] + s[1] > 10


def connection(s1, s2):
    # dumb function, return a false
    return (True, prm.euclid(s1, s2))


if __name__ == '__main__':
    main()

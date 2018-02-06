#!/usr/bin/env python

"""Create a PRM for a simple 2D state space. Exercice to get started with
ROS as part of the jalon 1."""

import os

try:
    import rospy
    import rospkg
    from obstacles.srv import CheckPoint, CheckConnection
except ImportError:
    print('ROS not runnning')

import prm

PACKAGE_NAME = 'roadmap'
CHECKPOINT_SRV = 'check_point'
CHECKCONN_SRV = 'check_connection'

rospackage = rospkg.RosPack()
PACKAGE_DATA_PATH = os.path.join(rospackage.get_path(PACKAGE_NAME), 'data')
if not os.path.exists(PACKAGE_DATA_PATH):
    os.makedirs(PACKAGE_DATA_PATH)

NB_SAMPLE = 1000
NB_CONECT = 3
# node: index of a node
# state: tuple representing the state associated or not with a node


def main():
    rospy.init_node('init_the_prm')
    rospy.loginfo('Init PRM initialized')
    rospy.wait_for_service(CHECKPOINT_SRV)
    rospy.wait_for_service(CHECKCONN_SRV)
    rospy.loginfo('End of wait for check_point')

    state_space = prm.StateSpace([(0, 10), (0, 5)], seed=0)
    graph = prm_init(state_space, NB_SAMPLE, NB_CONECT)
    # rospy.loginfo(graph)
    graph.save(os.path.join(PACKAGE_DATA_PATH))


def valid_state(s):
    """Call valid state service to check if the state is not in an obstacle"""
    try:
        # asking a new proxy every time does not slow down the process
        get_if_valid = rospy.ServiceProxy(CHECKPOINT_SRV, CheckPoint)
        resp = get_if_valid(*s)
        return resp.is_valid
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    # return s[0] + s[1] < 8 or s[0] + s[1] > 10


def connection(s1, s2):
    """Call valid connection service to check if the connection
    between the 2 states is possible"""
    try:
        # asking a new proxy every time does not slow down the process
        get_if_valid = rospy.ServiceProxy(CHECKCONN_SRV, CheckConnection)
        resp = get_if_valid(s1[0], s1[1], s2[0], s2[1])
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    return (resp.is_valid, prm.euclid(s1, s2))
    # return (True, prm.euclid(s1, s2))


if __name__ == '__main__':
    main()

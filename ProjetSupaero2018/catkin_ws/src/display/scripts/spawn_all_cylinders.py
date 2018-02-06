#!/usr/bin/env python

import rospy
import numpy as np
import spawn_cylinder_client
import sys

PARAM_NAME_SIZE = '/obstacles/obstacles_size'
PARAM_NAME_OBSTACLES = '/obstacles/obstacles_vec'
READ_OBSTACLES_SERVICE = 'read_obstacles'
SPAWN_CYLINDER_SERVICE = 'spawn_cylinder'

if __name__ == "__main__":
    rospy.wait_for_service(READ_OBSTACLES_SERVICE)
    rospy.wait_for_service(SPAWN_CYLINDER_SERVICE)

    try:
        vec = rospy.get_param(PARAM_NAME_OBSTACLES)
        size = rospy.get_param(PARAM_NAME_SIZE)
    except KeyError:
        rospy.logerr('Obstacles parameters not set - '
                     'You may have to launch a server that reads obstacles '
                     'from an input file '
                     '(see obstacles/read_obstacles_server)')
        sys.exit(1)

    n = len(vec) / size
    arr = np.reshape(np.array(vec), (n, size))

    for obstacle in arr:
        x = obstacle[0]
        y = obstacle[1]
        r = obstacle[2]
        spawn_cylinder_client.spawn_cylinder_client(x, y, r)

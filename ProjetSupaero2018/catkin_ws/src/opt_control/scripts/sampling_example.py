#!/usr/bin/env python
import numpy as np

import rospy

from opt_control.srv import Samples

rospy.init_node('sampling_client')
rospy.wait_for_service('create_samples')

try:
    create_samples = rospy.ServiceProxy('create_samples', Samples)

    print("Generating 30 samples:")
    resp = create_samples(30)
    samples = np.matrix(resp.samples).reshape(30, int(len(resp.samples)/30))
    print(samples)

except rospy.ServiceException as e:
    print("Service call failed: %s" % e)

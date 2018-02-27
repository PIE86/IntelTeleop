#!/usr/bin/env python
import rospy
from opt_control.srv import Samples

rospy.init_node('sampling_client')
rospy.wait_for_service('create_samples')

try:
    sampling_service = rospy.ServiceProxy('create_samples', Samples)

    print("Generating 30 samples:")
    resp = sampling_service(30)
    for sample in resp.samples:
        print(sample.x, sample.y, sample.z)

except rospy.ServiceException, e:
    print "Service call failed: %s" % e
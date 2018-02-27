#!/usr/bin/env python
import rospy
from opt_control.srv import Sample

rospy.init_node('sampling_client')
rospy.wait_for_service('create_sample')

try:
    sampling_service = rospy.ServiceProxy('create_sample', Sample)

    print("Generating 20 samples:")
    for _ in range(20):
        resp = sampling_service()
        print(resp.sample.x, resp.sample.y, resp.sample.z)

except rospy.ServiceException, e:
    print "Service call failed: %s" % e

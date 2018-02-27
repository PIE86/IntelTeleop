#!/usr/bin/env python
import rospy
from sampler.srv import Sample

rospy.init_node('sampler_client')
rospy.wait_for_service('sampler_service')
print("HEEEELLLLLOOOOO")

try:
    sampler_service = rospy.ServiceProxy('sampler_service', Sample)

    print("Generating 20 samples:")
    for _ in range(20):
        resp = sampler_service()
        print(resp.sample.x, resp.sample.y, resp.sample.z)

except rospy.ServiceException, e:
    print "Service call failed: %s" % e

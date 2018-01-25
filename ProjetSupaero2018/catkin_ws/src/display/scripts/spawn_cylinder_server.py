#!/usr/bin/env python

import rospy
from display.srv import SpawnCylinder, SpawnCylinderResponse
import rospkg
import subprocess
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion


# rosrun xacro xacro.py model.xacro > model.urdf
PACKAGE_NAME = 'display'
ROS_PACKAGE = rospkg.RosPack()
PACKAGE_DIR = ROS_PACKAGE.get_path(PACKAGE_NAME)
XACRO_PATH = PACKAGE_DIR + '/xacro/'
URDF_PATH = PACKAGE_DIR + '/urdf/'
model_count = 0


def spawn_cylinder(req):
    print "Adding cylinder: (%s, %s, %s)" \
          % (req.x, req.y, req.r)

    # Spawn Cylinder
    # Create URDF
    str_whitespace = str(' ')
    xacro_instruction = 'rosrun xacro xacro --inorder -o'
    xacro_path = XACRO_PATH + 'cylinder.xacro'
    xacro_args = 'x:=' + str(0) + str_whitespace \
                 + 'y:=' + str(0) + str_whitespace \
                 + 'radius:=' + str(req.r)
    urdf_path = URDF_PATH + 'cylinder.urdf'
    launch_xacro_instructions = str_whitespace.join([xacro_instruction,
                                                     urdf_path,
                                                     xacro_path,
                                                     xacro_args])

    try:
        # shell=True so that .bashrc is sourced
        subprocess.call(launch_xacro_instructions, shell=True)
    except OSError, e:
        rospy.logerr("Subprocess call failed: %s" % e)
        print "Subprocess call failed: %s" % e

    #  Call Gazebo service
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

    # args for spawn_model
    # model_name
    global model_count
    model_name = 'my_cylinder' + str(model_count)

    # model_xml
    with open(urdf_path, 'r') as urdf_file:
        model_xml = urdf_file.readlines()
        model_xml = ''.join(model_xml)

    # robot_namespace
    robot_namespace = 'gazebo'

    # initial_pose
    initial_pose = Pose()
    initial_pose.position = Point(req.x, req.y, 0.0)
    initial_pose.orientation = Quaternion(0, 0, 0, 1)

    # reference_frame
    reference_frame = 'world'

    spawn_model(model_name,
                model_xml,
                robot_namespace,
                initial_pose,
                reference_frame)

    model_count += 1
    rospy.loginfo(str("Spawned cylinder nr" + str(model_count)))

    return SpawnCylinderResponse(True)


def spawn_cylinder_server():
    rospy.init_node('spawn_cylinder_server')
    rospy.Service('spawn_cylinder', SpawnCylinder, spawn_cylinder)

    rospy.spin()


if __name__ == "__main__":
    spawn_cylinder_server()

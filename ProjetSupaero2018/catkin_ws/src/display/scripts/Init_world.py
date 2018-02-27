#!/usr/bin/env python
import os
import tf
import sys
import rospy
import subprocess
import numpy as np
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point, Quaternion, Pose

# car model name:
_car_name = 'my_car'
# get the relative path to display package
_path = os.path.realpath(__file__).split("scripts")[0]
# set to True to enable debug mode
_debug = True
# rosrun xacro xacro.py model.xacro > model.urdf
_cylinder_path = _path + 'models/cylinder/'
# count the total amount of obstacles
_model_count = 0


def spawn_car(pose0):

    with open(_path + "models/my_wheel/model.sdf", "r") as f:
        model_xml = f.read()

    angle = tf.transformations.quaternion_from_euler(
        pose0[3],
        pose0[4],
        pose0[5])
    orient = Quaternion(angle[0], angle[1], angle[2], angle[3])
    pose = Pose(Point(x=pose0[0], y=pose0[1], z=pose0[2]), orient)

    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_sdf_model(_car_name, model_xml, "", pose, "world")
    f.close()
    print('Car spawned as: '+_car_name)


def spawn_start_end_points(sartPose, endPose):

    startPointName = 'start_point'
    endPointName = 'end_point'

    with open(_path + "models/StartCone/model.sdf", "r") as f:
        model_xml = f.read()

    angle = tf.transformations.quaternion_from_euler(
        sartPose[3],
        sartPose[4],
        sartPose[5])
    orient = Quaternion(angle[0], angle[1], angle[2], angle[3])
    pose = Pose(Point(x=sartPose[0], y=sartPose[1], z=sartPose[2]), orient)

    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_sdf_model(startPointName, model_xml, "", pose, "world")
    f.close()
    print('Car start point set as: '+startPointName)

    with open(_path + "models/EndSign/model.sdf", "r") as f:
        model_xml = f.read()

    angle = tf.transformations.quaternion_from_euler(
        endPose[3],
        endPose[4],
        endPose[5])
    orient = Quaternion(angle[0], angle[1], angle[2], angle[3])
    pose = Pose(Point(x=endPose[0], y=endPose[1], z=endPose[2]), orient)

    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_sdf_model(endPointName, model_xml, "", pose, "world")
    f.close()
    print('End point set as: '+endPointName)


def creat_cylinder(radius):

    str_whitespace = str(' ')
    xacro_instruction = 'rosrun xacro xacro --inorder -o'
    xacro_path = _cylinder_path + 'cylinder.xacro'
    xacro_args = 'x:=' + str(0) + str_whitespace \
                 + 'y:=' + str(0) + str_whitespace \
                 + 'radius:=' + str(radius)
    urdf_path = _cylinder_path + 'cylinder.urdf'
    launch_xacro_instructions = str_whitespace.join([xacro_instruction,
                                                     urdf_path,
                                                     xacro_path,
                                                     xacro_args])
    try:
        # shell=True so that .bashrc is sourced
        subprocess.call(launch_xacro_instructions, shell=True)
    except OSError as e:
        rospy.logerr("Subprocess call failed: %s" % e)
    pass


def spawn_cylinder(x, y, radius):

    global _model_count
    creat_cylinder(radius)

    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    model_name = 'my_cylinder' + str(_model_count)

    # model_xml
    with open(_cylinder_path + 'cylinder.urdf', 'r') as cylinder_file:
        model_xml = cylinder_file.readlines()
        model_xml = ''.join(model_xml)

    # robot_namespace
    robot_namespace = ''

    # initial_pose
    initial_pose = Pose()
    initial_pose.position = Point(x, y, 0.0)
    initial_pose.orientation = Quaternion(0, 0, 0, 1)

    # reference_frame
    reference_frame = 'world'

    spawn_model(model_name,
                model_xml,
                robot_namespace,
                initial_pose,
                reference_frame)

    _model_count += 1
    rospy.loginfo(str("Spawned cylinder nr" + str(_model_count)))


def spawn_obstacles():

    PARAM_NAME_SIZE = 'utils/obstacles/obstacles_size'
    PARAM_NAME_OBSTACLES = 'utils/obstacles/obstacles_vec'
    READ_OBSTACLES_SERVICE = 'read_obstacles'

    rospy.wait_for_service(READ_OBSTACLES_SERVICE)

    try:
        vec = rospy.get_param(PARAM_NAME_OBSTACLES)
        size = rospy.get_param(PARAM_NAME_SIZE)
        if _debug:
            print('obstacles parameters found')
    except KeyError:
        rospy.logerr('Obstacles parameters not set - '
                     'You may have to launch a server that reads obstacles '
                     'from an input file '
                     '(see obstacles/read_obstacles_server)')
        if _debug:
            print('obstacles parameters not found')
        sys.exit(1)

    n = len(vec) / size
    obstacles = np.reshape(np.array(vec), (n, size))

    for obs in obstacles:
        spawn_cylinder(obs[0], obs[1], obs[2])


def initWorld():

    # initial car position and orientation as [x, y, z, alpha, beta, gamma]
    startPose = [0, 0, 0, 0, 0, 0]
    endPose = [2, -3, 0, 0, 0, 20]
    spawn_car(startPose)
    spawn_start_end_points(startPose, endPose)
    spawn_obstacles()


if __name__ == '__main__':

    try:
        initWorld()
    except rospy.ROSInterruptException:
        pass

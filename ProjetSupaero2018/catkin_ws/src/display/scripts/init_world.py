#!/usr/bin/env python
import os
import os.path as pp
import pyquaternion
import sys
import rospy
import subprocess
import numpy as np
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point, Quaternion, Pose


'''
Filed used to initialize the environment from various parameters
Currently, the world is generated with obstacles, a start point, an end point
and the car
'''

# Debug mode (== verbose)
DEBUG = True

# car model name:
CAR_NAME = 'my_car'

# get the relative path to display package
PATH_TO_DISPLAY_PKG = os.path.realpath(__file__).split("scripts")[0]

# path to the cylinder files
CYLINDER_PATH = pp.join(PATH_TO_DISPLAY_PKG, 'models', 'cylinder')

# count the total amount of obstacles generated
MODEL_COUNT = 0

# names from the obstacles package
PARAM_NAME_SIZE = 'obstacles/obstacles_size'
PARAM_NAME_OBSTACLES = 'obstacles/obstacles_vec'
READ_OBSTACLES_SERVICE = 'read_obstacles'


def build_model(path_to_model, pose_vector):
    """
    Get model parameters required to spawn it
    Should be used before spawn_model
    :param path_to_model: absolute path as a string
    :param pose_vector: pose as a vector [x, y, z, a1, a2, a3]
    :return: model_xml: xml read as a string,
             pose: pose as a geometry_msg::Pose(Point, Quaternion)
    """
    # Get model and read as string
    with open(path_to_model, "r") as f:
        model_xml = f.read()

    # Get pose from input vector
    angles = pose_vector[:3]
    # get quaternion from angle: quat=[w,x,y,z]
    quat = pyquaternion.Quaternion(axis=[0, 0, 1], angle=angles[-1])
    # build ros quaternion: rosquat=[x,y,z,w]
    orientation = Quaternion(quat[1],
                             quat[2],
                             quat[3],
                             quat[0])
    pose = Pose(Point(x=pose_vector[0],
                      y=pose_vector[1],
                      z=pose_vector[2]),
                orientation)

    return model_xml, pose


def spawn_model(model_name, model_xml, pose):
    """
    Spawn model using Gazebo service
    build_model returns some of the input parameters of this function
    :param model_name: name to be given in Gazebo
    :param model_xml: xml of the model
    :param pose: pose of the model as geometry_msg::Quaternion
    """
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_sdf_model(model_name, model_xml, "", pose, "world")
    if DEBUG:
        print('Model spawned as: ' + model_name)


def spawn_car(pose0):
    """
    Spawn car specifically, with given pose
    :param pose0: pose as geometry_msg::Quaternion
    """
    # Get model and read as string
    path_to_model = pp.join(PATH_TO_DISPLAY_PKG,
                            "models",
                            "my_wheel",
                            "model.sdf")
    # Get model params
    model_xml, pose = build_model(path_to_model, pose0)
    # Actually spawn model
    spawn_model(CAR_NAME, model_xml, pose)


def spawn_start_point(start_pose):
    """
    Spawn start point as a roadwork cone
    :param start_pose: pose as geometry_msg::Quaternion
    """
    start_point_name = 'start_point'
    path_to_mdl = pp.join(PATH_TO_DISPLAY_PKG,
                          "models",
                          "StartCone",
                          "model.sdf")
    model_xml, pose = build_model(path_to_mdl, start_pose)
    spawn_model(start_point_name, model_xml, pose)
    print('Car start point set as: '+start_point_name)


def spawn_end_point(end_pose):
    """
    Spawn end point as a stop sign
    :param end_pose: pose as geometry_msg::Quaternion
    """
    end_point_name = 'end_point'
    path_to_mdl = pp.join(PATH_TO_DISPLAY_PKG,
                          "models",
                          "EndSign",
                          "model.sdf")
    model_xml, pose = build_model(path_to_mdl, end_pose)
    spawn_model(end_point_name, model_xml, pose)
    print('End point set as: '+end_point_name)


def create_cylinder_urdf(radius):
    """
    Generate urdf file of cylinder with given radius, using xacro
    :param radius: radius of the cylinder
    :return: path to urdf file
    """
    # Save string for concision
    str_whitespace = str(' ')

    # Set up xacro instructions and args
    xacro_instruction = 'rosrun xacro xacro --inorder -o'
    xacro_path = CYLINDER_PATH + 'cylinder.xacro'
    xacro_args = 'x:=' + str(0) + str_whitespace \
                 + 'y:=' + str(0) + str_whitespace \
                 + 'radius:=' + str(radius)

    # Set up urdf path (to be returned)
    urdf_path = CYLINDER_PATH + 'cylinder.urdf'

    # Execute xacro instruction
    launch_xacro_instructions = str_whitespace.join([xacro_instruction,
                                                     urdf_path,
                                                     xacro_path,
                                                     xacro_args])
    try:
        # shell=True so that bashrc is sourced
        subprocess.call(launch_xacro_instructions, shell=True)
    except OSError as e:
        rospy.loginfo("Subprocess call failed: %s" % e)
        return 0  # as failure

    return urdf_path  # as success


def spawn_cylinder(x, y, radius):
    """
    Spawn cylinder with given radius and at given position
    :param x: position along x-axis (red)
    :param y: position along y-axis (green)
    :param radius: radius of the cylinder
    In addition, count the number of cylinder spawned
    """
    # Count number of cylinders
    global MODEL_COUNT

    # Model path to urdf and name
    model_path = create_cylinder_urdf(radius)
    model_name = 'my_cylinder_' + str(MODEL_COUNT)

    # Pose to be given, as vector
    pose_vector = [x, y, 0,  # position
                   0, 0, 0]  # orientation
    model_xml, pose = build_model(model_path, pose_vector)

    # Spawn
    spawn_model(model_name, model_xml, pose)

    MODEL_COUNT += 1
    rospy.loginfo(str("Spawned cylinder nr" + str(MODEL_COUNT)))


def spawn_obstacles():
    """
    Sequentially spawn obstacles found in param server, as cylinders
    """
    # Ensure obstacles are read (blocking instruction)
    rospy.wait_for_service(READ_OBSTACLES_SERVICE)

    # READ_OBSTACLES_SERVICES should set these param in ros param server
    try:
        vec = rospy.get_param(PARAM_NAME_OBSTACLES)
        size = rospy.get_param(PARAM_NAME_SIZE)
        if DEBUG:
            print('Obstacles parameters found')
    except KeyError:
        rospy.loginfo('Obstacles parameters not set - '
                      'You may have to launch a server that reads obstacles '
                      'from an input file '
                      '(see obstacles/read_obstacles_server)')
        if DEBUG:
            print('Obstacles parameters not found')
        sys.exit(1)

    # Convert list of obstacles into array
    n = int(len(vec) / size)
    obstacles = np.reshape(np.array(vec), (n, size))

    # Spawn each obstacle
    for obs in obstacles:
        spawn_cylinder(obs[0], obs[1], obs[2])


def init_world():
    """
    Spawn car, start and end points, and obstacles
    """
    # Initial car pose as [x, y, z, a1, a2, a3]
    start_pose = [4, 6, 0, 0, 0, 1]
    end_pose = [12, 8, 0, 0, 0, 1]

    spawn_car(start_pose)
    spawn_start_point(start_pose)
    spawn_end_point(end_pose)
    spawn_obstacles()


if __name__ == '__main__':
    try:
        init_world()
    except rospy.ROSInterruptException:
        pass

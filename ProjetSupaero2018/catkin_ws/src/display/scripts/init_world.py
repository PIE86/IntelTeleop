#!/usr/bin/env python
import os
import os.path as pp
import pyquaternion
import sys
import rospy
import subprocess
import numpy as np
from display.msg import State
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point, Quaternion, Pose

# set to True to enable debug mode (verbose)
DEBUG = True

# car model name:
CAR_NAME = 'my_car'
# get the relative path to display package
PATH_TO_DISPLAY_PKG = os.path.realpath(__file__).split("scripts")[0]
# rosrun xacro xacro.py model.xacro > model.urdf
CYLINDER_PATH = PATH_TO_DISPLAY_PKG + 'models/cylinder/'
# count the total amount of obstacles
MODEL_COUNT = 0

PARAM_NAME_SIZE = 'obstacles/obstacles_size'
PARAM_NAME_OBSTACLES = 'obstacles/obstacles_vec'
READ_OBSTACLES_SERVICE = 'read_obstacles'

END_STATE_TOPIC = 'end_state'
ESPS = 2  # Hz


class World:

    def __init__(self):
        # initial car position and orientation as [x, y, z, alpha, beta, gamma]

        # Easy one
        self.start_pose = [2, 2, 0, 0, 0, 2]
        self.end_pose = [12, 4, 0, 0, 0, 0]

        # Long one
        # self.start_pose = [2, 2, 0, 0, 0, 0]
        # self.end_pose = [15, 15, 0, 0, 0, np.pi/2]

        # Long and hard
        # self.start_pose = [2, 2, 0, 0, 0, np.pi/3]
        # self.end_pose = [11, 16, 0, 0, 0, 0]

        self.pub = rospy.Publisher(END_STATE_TOPIC, State, queue_size=10)

    def init_world(self):
        spawn_car(self.start_pose)
        spawn_start_point(self.start_pose)
        spawn_end_point(self.end_pose)
        spawn_obstacles()

    def send_end_state(self):
        self.pub.publish([self.end_pose[0],
                          self.end_pose[1],
                          self.end_pose[5],
                          ])


def build_model(path_to_model, pose_vector):
    """ Get model parameters required to spawn it"""
    """ Namely, from path and pose_vector, get:
     model as string (xml syntax)
     pose as Pose(Point, Quaternion)"""
    # Get model and read as string
    with open(path_to_model, "r") as f:
        model_xml = f.read()

    # Get pose
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
    """ Spawns model using Gazebo service """
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_sdf_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    spawn_sdf_model(model_name, model_xml, "", pose, "world")
    print('Model spawned as: ' + model_name)


def spawn_car(pose0):
    """ Spawn car specifically """
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
    start_point_name = 'start_point'
    path_to_mdl = pp.join(PATH_TO_DISPLAY_PKG,
                          "models",
                          "StartCone",
                          "model.sdf")
    model_xml, pose = build_model(path_to_mdl, start_pose)
    spawn_model(start_point_name, model_xml, pose)
    print('Car start point set as: '+start_point_name)


def spawn_end_point(end_pose):
    end_point_name = 'end_point'
    path_to_mdl = pp.join(PATH_TO_DISPLAY_PKG,
                          "models",
                          "EndSign",
                          "model.sdf")
    model_xml, pose = build_model(path_to_mdl, end_pose)
    spawn_model(end_point_name, model_xml, pose)
    print('End point set as: '+end_point_name)


def create_cylinder_urdf(radius):
    # Set up xacro instructions and args
    xacro_instruction = 'rosrun xacro xacro --inorder -o'
    xacro_path = CYLINDER_PATH + 'cylinder.xacro'
    xacro_args = 'x:=' + str(0) + ' ' \
                 + 'y:=' + str(0) + ' ' \
                 + 'radius:=' + str(radius)

    # Set up urdf path
    urdf_path = CYLINDER_PATH + 'cylinder.urdf'

    # Execute xacro instruction
    launch_xacro_instructions = ' '.join([xacro_instruction, urdf_path,
                                          xacro_path, xacro_args])
    try:
        # shell=True so that .bashrc is sourced
        subprocess.call(launch_xacro_instructions, shell=True)
    except OSError as e:
        rospy.loginfo("Subprocess call failed: %s" % e)
        return 0  # as failure

    return urdf_path  # as success


def spawn_cylinder(x, y, radius):
    global MODEL_COUNT
    model_path = create_cylinder_urdf(radius)
    model_name = 'my_cylinder_' + str(MODEL_COUNT)

    pose_vector = [x, y, 0,  # position
                   0, 0, 0]  # orientation
    model_xml, pose = build_model(model_path, pose_vector)

    # reference_frame
    spawn_model(model_name, model_xml, pose)

    MODEL_COUNT += 1
    rospy.loginfo(str("Spawned cylinder nr" + str(MODEL_COUNT)))


def spawn_obstacles():
    rospy.wait_for_service(READ_OBSTACLES_SERVICE)

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

    n = int(len(vec) / size)
    obstacles = np.reshape(np.array(vec), (n, size))

    for obs in obstacles:
        spawn_cylinder(obs[0], obs[1], obs[2])


if __name__ == '__main__':
    rospy.init_node('task', anonymous=True)
    world = World()
    world.init_world()
    rate = rospy.Rate(ESPS)
    while not rospy.is_shutdown():
        world.send_end_state()
        rate.sleep()

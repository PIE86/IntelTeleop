#!/usr/bin/env python
import os
import sys
import time
import numpy as np
import os.path as pp
import pyquaternion
import subprocess
import rospy
from display.msg import State
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point, Quaternion, Pose


"""
Filed used to initialize the environment from various parameters
Currently, the world is generated with obstacles, a start point, an end point
and the car.
Commentaries associated with each start/end correspond to the weights saved
in github.
"""

# Debug mode (== verbose)
DEBUG = True

# get the relative path to display package
PATH_TO_DISPLAY_PKG = os.path.realpath(__file__).split("scripts")[0]

# path to the cylinder files
CYLINDER_PATH = pp.join(PATH_TO_DISPLAY_PKG, 'models', 'cylinder')

# names from the obstacles package
PARAM_NAME_SIZE = 'obstacles/obstacles_size'
PARAM_NAME_OBSTACLES = 'obstacles/obstacles_vec'
READ_OBSTACLES_SERVICE = 'read_obstacles'

END_STATE_TOPIC = 'end_state'
ESPS = 2  # Hz
SECOND_END = False

# count the total amount of obstacles generated
obstacle_nb = 0


class World:

    def __init__(self):
        # initial car position and orientation as [x, y, z, alpha, beta, gamma]

        # 180 ° -> ok without init
        # self.start_pose = [2, 2, 0, 0, 0, 0]
        # self.end_pose = [2, 2, 0, 0, 0, np.pi]

        # Easy one -> long without init
        # self.start_pose = [2, 2, 0, 0, 0, 0]
        # self.end_pose = [14, 4, 0, 0, 0, 0]

        # Long one -> long without init
        # self.start_pose = [2, 2, 0, 0, 0, 0]
        # self.end_pose = [15, 15, 0, 0, 0, np.pi/2]

        # obstacle, v1 -> fail without init
        # self.start_pose = [5, 11, 0, 0, 0, 0]
        # self.end_pose = [14, 14, 0, 0, 0, 1.5]

        # obstacle, v2 -> fail without init
        # self.start_pose = [10, 8, 0, 0, 0, 0]
        # self.end_pose = [10, 15, 0, 0, 0, 3]

        # obstacle positive angle -> fail without init
        self.start_pose = [2, 2, 0, 0, 0, np.pi/2]
        self.end_pose = [11, 15, 0, 0, 0, 0]

        # NOPE for the 2
        # self.start_pose = [2, 2, 0, 0, 0, np.pi/2]
        # self.end_pose = [10, 17, 0, 0, 0, 0]

        self.pub = rospy.Publisher(END_STATE_TOPIC, State, queue_size=10)
        # used to spwan the second end once if needed
        self.spawned_end2 = False

    def init_world(self):
        spawn_element(self.start_pose, 'my_wheel', 'car')
        spawn_element(self.start_pose, 'StartCone', 'start')
        spawn_element(self.end_pose, 'EndSign', 'end1')
        spawn_obstacles()

    def start(self):
        rate = rospy.Rate(ESPS)
        t1 = time.time()
        while not rospy.is_shutdown():
            t2 = time.time()
            # TODO: Change end is hard coded but should be part of the GUI
            if SECOND_END and not self.spawned_end2 and (t2-t1) > 30:
                self.new_end()
                self.spawned_end2 = True
            self.send_end_state()
            rate.sleep()

    def send_end_state(self):
        self.pub.publish([self.end_pose[0],
                          self.end_pose[1],
                          self.end_pose[5],
                          ])

    def new_end(self):
        self.end_pose = [16, 13, 0, 0, 0, np.pi/2]
        spawn_element(self.end_pose, 'EndSign', 'end2')


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
    with open(path_to_model, 'r') as f:
        model_xml = f.read()

    # Get pose from input vector
    angles = pose_vector[-3:]
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


def spawn_element(pose, model_name, elt_name):

    # Get model and read as string
    path_to_model = pp.join(PATH_TO_DISPLAY_PKG,
                            'models',
                            model_name,
                            'model.sdf')
    # Get model params
    model_xml, pose = build_model(path_to_model, pose)
    # Actually spawn model
    spawn_model(elt_name, model_xml, pose)


def spawn_model(model_name, model_xml, pose):
    """
    Spawn model using Gazebo service
    build_model returns some of the input parameters of this function
    :param model_name: name to be given in Gazebo
    :param model_xml: xml of the model
    :param pose: pose of the model as geometry_msg::Quaternion
    """
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_sdf_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_sdf_model(model_name, model_xml, '', pose, 'world')
    print('Model spawned as: ' + model_name)


def create_cylinder_urdf(radius):
    """
    Generate urdf file of cylinder with given radius, using xacro
    :param radius: radius of the cylinder
    :return: path to urdf file
    """
    # Set up xacro instructions and args
    xacro_instruction = 'rosrun xacro xacro --inorder -o'
    xacro_path = pp.join(CYLINDER_PATH, 'cylinder.xacro')
    xacro_args = 'x:=' + str(0) + ' ' \
                 + 'y:=' + str(0) + ' ' \
                 + 'radius:=' + str(radius)

    # Set up urdf path (to be returned)
    urdf_path = CYLINDER_PATH + 'cylinder.urdf'

    # Execute xacro instruction
    launch_xacro_instructions = ' '.join([xacro_instruction, urdf_path,
                                          xacro_path, xacro_args])
    try:
        # shell=True so that bashrc is sourced
        subprocess.call(launch_xacro_instructions, shell=True)
    except OSError as e:
        rospy.loginfo('Subprocess call failed: %s' % e)
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
    global obstacle_nb

    # Model path to urdf and name
    model_path = create_cylinder_urdf(radius)
    model_name = 'my_cylinder_' + str(obstacle_nb)

    # Pose to be given, as vector
    pose_vector = [x, y, 0,  # position
                   0, 0, 0]  # orientation
    model_xml, pose = build_model(model_path, pose_vector)

    # Spawn
    spawn_model(model_name, model_xml, pose)

    obstacle_nb += 1
    rospy.loginfo(str('Spawned cylinder nr' + str(obstacle_nb)))


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


if __name__ == '__main__':
    rospy.init_node('task', anonymous=True)
    world = World()
    world.init_world()
    world.start()

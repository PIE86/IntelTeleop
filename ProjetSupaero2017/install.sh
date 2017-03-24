#!/bin/bash

mv ./quadrotor_base.urdf.xacro catkin_ws/src/hector_quadrotor/hector_quadrotor_description/urdf/quadrotor_base.urdf.xacro
mv ./quadrotor_propulsion.gazebo.xacro catkin_ws/src/hector_quadrotor/hector_quadrotor_gazebo/urdf/quadrotor_propulsion.gazebo.xacro
mv ./spawn_quadrotor.launch catkin_ws/src/hector_quadrotor/hector_quadrotor_gazebo/launch/spawn_quadrotor.launch

cd catkin_ws/src
catkin_init_workspace

# to execute when in catkin_ws/src folder
sudo apt-get install ros-kinetic-ros-control
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-unique-identifier
sudo apt-get install ros-kinetic-geographic-info
sudo apt-get install ros-kinetic-laser-geometry
sudo apt-get install ros-kinetic-tf-conversions
sudo apt-get install ros-kinetic-tf2-geometry-msgs
sudo apt-get install ros-kinetic-joy

cd ..
catkin_make

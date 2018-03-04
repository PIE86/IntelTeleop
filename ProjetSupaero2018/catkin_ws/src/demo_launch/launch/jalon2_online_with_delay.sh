#!/bin/bash

# Init obstacles
echo "Launching obstacles"
echo "........................................................................"
roslaunch obstacles obstacles.launch &
sleep 3s

# Wait 3s
echo "........................................................................"
read -p "Waiting to launch Gazebo" -n 1 -s
echo "Launching Gazebo"
echo "........................................................................"

# Init Gazebo
roslaunch display display.launch &
sleep 40s

# Wait 10s
echo "........................................................................"
read -n "Waiting to launch control" 1 -s
echo "Launching control"
echo "........................................................................"

# Init control
roslaunch opt_control online_control.launch

# IntelTeleop - User and Developper Guide


## Installation


Theses instructions will guide you through installing IntelTeleop

### Prerequisites

ROS Kinetic (full install) is required. See http://wiki.ros.org/ROS/Installation
For example for Ubuntu, that would be:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
```

ACADO Toolkit is also required.

```
sudo apt-get install gcc g++ cmake git gnuplot doxygen graphviz

git clone https://github.com/acado/acado.git -b stable ACADOtoolkit && cd ACADOtoolkit
git reset --hard 88c441b6bedee039ef8cb81d34fcd9377fb6d138
mkdir build && cd build
cmake .. && make
sudo make install
```
Le dossier ACADOtoolkit peut ensuite être supprimé.
0.

### Downloading IntelTeleop

Download the IntelTeleop project from our github where you want it to be:

```
git clone https://github.com/DianeBury/IntelTeleop.git
```

### Installing

Go in the 2017 project subfolder and launch the install script:

```
cd ProjetSupaero2017
sh install.sh
```

The script will setup your catkin workspace and will download the Hector Quadrotor package.

* You will have to source your catkin workspace with every new terminal. You can add it to your bashrc to make it automatic:

```
echo 'source ~/path/to/IntelTeleop/ProjectSupaero2017/catkin_ws/devel/setup.bash' >> ~/.bashrc
```


## User guide

First, launch a roscore in a new terminal:

```
roscore
```

In another terminal, run on of the following commands to run the demo using the keyboard (default) or the joystick:

```
roslaunch intel_teleop_demo demo.launch 
```

```
roslaunch intel_teleop_demo demo.launch joystick:=true
```

This will launch Gazebo. You will see the drone in an empty environnement, with only the ground and no obstacle. A terminal with the control node will open from which you can control the drone using the keyboard or the joystick depending on your choice. You should be able to control the drone in altitude and in position. See following section to add obstacles.


### Managing the environnement

For now, you can only add vertical and infinite (not visually) cylinders. To do so, during run-time, use the following command:

```
rosservice call /addCylinder -- radius x1 y1 z1 x2 y2 z2
```
Where radius is the radius, x-y-z1 are the coordinates of the first circle of the cylinder and x-y-z2 the coordinates of the second circle. At the moment, you should only use it with vertical cylinders (x-y1 == x-y2 and z1 different from z2), because the internal avoidance algorithm is unaware of other types of cylinders.

For example:

```
rosservice call /addCylinder -- 1.0 10.0 0.0 0.0 10.0 0.0 5.0
```
Will create a 5m high cylinder with a 1m radius at the position 10, 0.

You can also add wind to the simulation with the command:

```
rostopic pub -r 50 /wind geometry_msgs/Vector3 -- x y z
```

Which adds wind in the direction (x, y, z), with a force proportional to |(x, y, z)|.

## Developper guide

### Launch file description

Launching the demo.launch file loads the environnement description (world file) and creates several nodes to simulate and control the drone.

```
<launch>
  <!-- Launch Hector Quadrotor simulation and IntelTeleop optimal control nodes -->
  
  <!-- get user argument to use joystick or not (keyboard is default) -->
  <arg name="joystick" default="false"/>

  <!-- load world file -->
  <include file="$(find intel_teleop_demo)/launch/demo_world.launch" >
  </include>

  <!-- spawn hector quadrotor -->
  <include file="$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor.launch" >
  </include>

  <!-- launch user interface node -->
  <include file="$(find intel_teleop_demo)/launch/command_interface.launch" >
    <arg name="joy_dev" value="/dev/input/js2" />
    <arg name="joystick" value="$(arg joystick)" />
  </include>

  <!-- call script calling service to enable motors -->
  <node name="enable_motors" pkg="intel_teleop_demo" type="enable_motors.sh" output="screen" />

  <!-- launch Hector Quadrotor pose estimator node -->
  <node name="pose_estimator" pkg="hector_quadrotor_pose_estimation" type="hector_quadrotor_pose_estimation" output="screen" />

  <!-- launch IntelTeleop optimal control -->
  <node name="opt_control" pkg="intel_teleop_optcontrol" type="optimal_control" output="screen" />

  <!-- start rviz visualization - optional
  <node pkg="rviz" type="rviz" name="rviz" args="-f $(find hector_quadrotor_demo)/rviz_cfg/outdoor_flight.vcg" /> -->

</launch>
```

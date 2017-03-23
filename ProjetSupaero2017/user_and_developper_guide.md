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

This will launch Gazebo. You will see the drone surrounded by a test environnement. A terminal with the control node will open from which you can control the drone using the keyboard or the joystick depending on your choice. You should see how the drone avoids the obstacles as it flies.

## Code architecture

### Launch file description

Launching the demo.launch file creates several nodes to simulate and control the drone. We can look more closely at the launch file :

```
<launch>
  <!-- launches Hector Quadrotor simulation and IntelTeleop optimal control nodes -->
  <arg name="joystick" default="false"/>

  <!-- world file -->
  <include file="$(find intel_teleop_demo)/launch/demo_world.launch" >
  </include>

  <!-- hector quadrotor -->
  <include file="$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor.launch" >
  </include>

  <!-- teleoperation -->
  <include file="$(find intel_teleop_demo)/launch/command_interface.launch" >
    <arg name="joy_dev" value="/dev/input/js2" />
    <arg name="joystick" value="$(arg joystick)" />
  </include>

  <!-- parameters -->
  <include file="$(find intel_teleop_demo)/launch/parameters_demo.launch" />

  <!-- script calling service to enable motors -->
  <node name="enable_motors" pkg="intel_teleop_demo" type="enable_motors.sh" output="screen" />

  <!-- Pose Estimator -->
  <node name="pose_estimator" pkg="hector_quadrotor_pose_estimation" type="hector_quadrotor_pose_estimation" output="screen" />

  <!-- Optimal Control -->
  <node name="opt_control" pkg="intel_teleop_optcontrol" type="optimal_control" output="screen" />

  <!-- start rviz visualization 
  <node pkg="rviz" type="rviz" name="rviz" args="-f $(find hector_quadrotor_demo)/rviz_cfg/outdoor_flight.vcg" /> -->

</launch>
```



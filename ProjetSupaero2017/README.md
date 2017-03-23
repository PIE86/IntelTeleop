# IntelTeleop

This project implements optimal control for a smart teleoperation for a drone as ROS modules. It uses the Hector Quadrotor package for the simulation through Gazebo.

## Getting started

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

The script will init the catkin_workspace, download the Hector Quadrotor package (which can take some time) then build everything.

Now source your catkin workspace with the following command when in catkin_ws:

```
source devel/setup.bash
```

* You will have to source your catkin workspace with every new terminal. You can add it to your bashrc to make it automatic:

```
echo 'source ~/path/to/IntelTeleop/ProjectSupaero2017/catkin_ws/devel/setup.bash' >> ~/.bashrc
```

## Running the demo

In a terminal, run on of the following commands to run the demo using the keyboard (default) or the joystick:

```
roslaunch intel_teleop_demo demo.launch 
```

```
roslaunch intel_teleop_demo demo.launch joystick:=true
```

This will launch Gazebo. You will see the drone surrounded by a test environnement. A terminal with the control node will open from which you can control the drone using the keyboard or the joystick depending on your choice. You should see how the drone avoids the obstacles as it flies.

## Example

## Authors

* **Diane Bury**
* **Alexis Nicolin**
* **Bertrand Suderie**
* **Steve Ravalisse**
* **Andrea Brugnoli**
* **Paolo Panicucci**

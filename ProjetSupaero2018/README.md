# Installation
This project was tested with
* Ubuntu 16.04
* ROS Kinetic, Lunar
* python 3.6
* python libraries versions as in `requirements.txt`

## Project download
git clone https://github.com/PIE86/MemoryEnhancedPredictiveControl.git  

## ROS Kinetic
(full install) is required. See http://wiki.ros.org/ROS/Installation  
For example for Ubuntu, that would be:  

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
```

## ACADO Toolkit

```
sudo apt-get install gcc g++ cmake git gnuplot doxygen graphviz

git clone https://github.com/acado/acado.git -b stable ACADOtoolkit && cd ACADOtoolkit
git reset --hard 88c441b6bedee039ef8cb81d34fcd9377fb6d138
mkdir build && cd build
cmake .. && make
sudo make install
```

Add the models directory to your Gazebo model path
```
export GAZEBO_MODEL_PATH=$(rospack find display)/models:$GAZEBO_MODEL_PATH
```

You might want to add this line to your bashrc, after sourcing the ROS environment.
You can also specify the full path. In the future, we hope that we will be able to do otherwise (without being constraining for the user)

## Setting up your python environment
For this project, python 3.6 was tested  
### Using Conda
If you did not install conda, visit [this page](https://conda.io/docs/user-guide/install/linux.html#install-linux-silent).  
Choose anaconda if you have sufficient free memory on your computer (>2Go), otherwise miniconda.   
Create a virtual environment using python 3.6:   
`conda create -n <the_name_of_your_choice> python=3.6`  
Replace <the_name_of_your_choice> with irepa for example.  
To activate your environment (do this every time you work on the project): `source activate irepa`   

### Using virtualenv
Install python3.6: `sudo apt-get install python3.6`  
Install virtualenv: `pip install --user virtualenv`  
Install virtualenvwrapper: `pip install --user virtualenvwrapper`  
Add these lines at the end of your .bashrc:   
`export WORKON_HOME=~/.virtualenvs`  
`mkdir -p $WORKON_HOME`  
`source ~/.local/bin/virtualenvwrapper.sh`  
Depending on your installation, `virtualenvwrapper.sh` might not be in
`~/.local/bin/`. Make sure it is the right path and if not change your path accordingly.

Now create an environment: `mkvirtualenv irepa`  
Activate your environment: `workon irepa`

### Install python packages
In ProjetSupaero2018 directory  
`pip install -r requirements.txt`  

# Demo
## Build the project
In ProjetSupaero2018/catkin_ws run  
`catkin_make`

## Add project path to ROS_PACKAGE_PATH:
In ProjetSupaero2018/catkin_ws run  
`source devel/setup.bash`

## launch demos
For all launches, if you do not need logs in your shell, remove --screen at the
end of the commands. jalon2 version of the project has default network weights that will be overrided when a new training is undergone.   
### Online Simulation
Open gazebo and launch the simulation. Start and end states can be changed in
src/display/scripts/init_world.py
`roslaunch demo_launch jalon2_online.launch --screen`

### IREPA training
`roslaunch demo_launch jalon2_irepa_train.launch --screen`

If roslaunch does not find demo_launch package, try to first execute  
`roscd demo_launch`  


# Git
When doing a pull request, choose "develop" as the target branch and PAY ATTENTION to
choose right repo for develop (default is Diane's repo)

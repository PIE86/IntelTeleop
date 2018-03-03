# Installation
## Requirements
This project was tested with
* Ubuntu 16.04
* ROS Kinetic, Lunar
* python 3.6
* python libraries versions as in `requirements.txt`

## ROS
TODO

##ACADO
TODO

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

Now create an environment:  `mkvirtualenv irepa`  
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

## launch demo
If roslaunch does not find demo_launch package, try to first execute  
`roscd demo_launch`  

### Jalon 1
TODO
`roslaunch demo_launch jalon1_demo.launch`


### Jalon 2
#### Online Simulation
DESCRIPTION  
`roslaunch demo_launch jalon2_online.launch`


?????????????????
### add the models directory to your Gazebo model path
export GAZEBO_MODEL_PATH=$(rospack find display)/models:$GAZEBO_MODEL_PATH

You might want to add this line to your bashrc, after sourcing the ROS environment. You can also specify the full path. In the future, we hope that we will be able to do otherwise (without being constraining for the user)


# Developpers advices
### Git
When doing a pull request, choose "develop" as the target branch and PAY ATTENTION to
choose right repo for develop (default is Diane's repo)

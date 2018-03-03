# Installation
##ACADO
TODO

##Set up a virtual environement
### Using Conda
If you did not install conda, visit [this](https://conda.io/docs/user-guide/install/linux.html#install-linux-silent).  
Choose anaconda if you have sufficient free memory on your computer (>2Go), otherwise miniconda.   
Create a virtual environment using python 3.6:   
`conda create -n <the_name_of_your_choice> python=3.6`  
Replace <the_name_of_your_choice> with irepa for example.  
To activate your environment (do this every time you work on the project): `source activate irepa`   
Then install the python packages, in ProjetSupaero2018:  
`pip install -r requirements.txt`  

### Using virtualenv
TODO
http://sametmax.com/les-environnement-virtuels-python-virtualenv-et-virtualenvwrapper/


# Reminders
## Add project path to ROS_PACKAGE_PATH:
In catkin_ws directory of 2018 project, run `source devel/setup.bash`

### Git
When doing a pull request, choose "develop" as the target branch and PAY ATTENTION to
choose right repo for develop (default is Diane's repo)


## launch demo
The demo for J1 might be launched using the following commands:

### add the models directory to your Gazebo model path
export GAZEBO_MODEL_PATH=$(rospack find display)/models:$GAZEBO_MODEL_PATH

You might want to add this line to your bashrc, after sourcing the ROS environment. You can also specify the full path. In the future, we hope that we will be able to do otherwise (without being constraining for the user)

### build your catkin_ws
using catkin_make from the workspace

### roslaunch
roslaunch demo_launch jalon1_demo.launch

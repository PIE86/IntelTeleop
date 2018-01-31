TODO



## Reminders
### setup
#### Add project path to ROS_PACKAGE_PATH:
In catkin_ws directory of 2018 project, run `source devel/setup.bash`

### Git
When doing a pull request, choose "develop" as the target branch and PAY ATTENTION to 
choose right repo for develop (default is Diane's repo)


## launch demo
The demo for J1 might be launched using the following commands:

### add the models directory to your GAZEBO_MODEL_PATH 
export GAZEBO_MODEL_PATH=$HOME/MemoryEnhancedPredictiveControl/ProjetSupaero2018/catkin_ws/src/display/models:$GAZEBO_MODEL_PATH
You might want to add this line to your bashrc

## build your catkin_ws
using catkin_make from the workspace

## roslaunch
roslaunch demo_launch jalon1_demo.launch

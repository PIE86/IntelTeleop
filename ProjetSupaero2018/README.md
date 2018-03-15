# Installation

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

### Add the models directory to your Gazebo model path
```
export GAZEBO_MODEL_PATH=$(rospack find display)/models:$GAZEBO_MODEL_PATH
```

You might want to add this line to your bashrc, after sourcing the ROS environment. You can also specify the full path. In the future, we hope that we will be able to do otherwise (without being constraining for the user)

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


## Install the demo

```
git clone https://github.com/PIE86/MemoryEnhancedPredictiveControl.git
cd MemoryEnhancedPredictiveControl/ProjetSupaero2018/
catkin_make
source devel/setup.bash
```

## And launch it
```
roslaunch demo_launch jalon1_demo.launch
```

### Git
When doing a pull request, choose "develop" as the target branch and PAY ATTENTION to choose right repo for develop (default is Diane's repo)

cmake_minimum_required(VERSION 2.8.3)
project(intel_teleop_optcontrol)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  intel_teleop_msgs
)

FIND_PACKAGE( ACADO REQUIRED )

add_definitions(-std=c++11)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   addCylinderOptControl.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
#
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES intel_teleop_optcontrol
#  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(include
   ${roscpp_INDLUDE_DIRS}
   ${catkin_INCLUDE_DIRS}
        ${ACADO_INCLUDE_DIRS}
)

#add_library(baro src/baro.cpp)
#target_link_libraries(baro ${roscpp_LIBRARIES} ${catkin_LIBRARIES})

add_library(optcontrol src/optcontrol.cpp )
target_link_libraries(optcontrol ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${ACADO_SHARED_LIBRARIES})
add_dependencies( optcontrol intel_teleop_msgs_generate_messages_cpp )

add_executable(optimal_control src/optimal_control.cpp)
target_link_libraries(optimal_control ${catkin_LIBRARIES} ${roscpp_LIBRARIES} optcontrol)
add_dependencies(optimal_control ${catkin_EXPORTED_TARGETS})




cmake_minimum_required(VERSION 2.8.3)
project(display)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  utils
)

## Generate messages in the 'msg' folder
# add_message_files(
#  DIRECTORY msg
#  FILES 
#  Control.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
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
# )

catkin_package(
   CATKIN_DEPENDS
   utils
#  INCLUDE_DIRS include
#  LIBRARIES display
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

####################
## Gazebo Plugins ##
####################

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(car_control SHARED plugins/car_control_plugin.cc)
target_link_libraries(car_control ${GAZEBO_LIBRARIES})

add_library(state_feedback SHARED plugins/state_feedback_plugin.cc)
target_link_libraries(state_feedback ${GAZEBO_LIBRARIES})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

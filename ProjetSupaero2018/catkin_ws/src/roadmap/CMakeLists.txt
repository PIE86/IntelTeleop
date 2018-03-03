cmake_minimum_required(VERSION 2.8.3)
project(roadmap)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  genmsg actionlib_msgs actionlib
)

add_service_files(
   FILES
   PathFinding.srv
)

add_action_files(
  DIRECTORY action
  FILES OptControl.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
 # LIBRARIES car_model
 CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

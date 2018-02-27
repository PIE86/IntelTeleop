cmake_minimum_required(VERSION 2.8.3)
project(utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY ros_com/msg
   FILES
   Command.msg
   State.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

catkin_package(
   CATKIN_DEPENDS
   message_runtime
   geometry_msgs 
   std_msgs
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
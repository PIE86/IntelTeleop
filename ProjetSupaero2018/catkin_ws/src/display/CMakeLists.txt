cmake_minimum_required(VERSION 2.8.3)
project(display)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

 add_service_files(
   FILES
   SpawnCylinder.srv
 )
 
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

 catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES display
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

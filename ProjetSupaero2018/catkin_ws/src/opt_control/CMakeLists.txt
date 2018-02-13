cmake_minimum_required(VERSION 2.8.3)
project(opt_control)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

add_service_files(
  FILES
  OptControl.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 # LIBRARIES car_model
 CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs message_runtime
)

FIND_PACKAGE(ACADO REQUIRED)

add_definitions(-std=c++11)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${roscpp_INDLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${ACADO_INCLUDE_DIRS}
)

add_executable(acado_opt_control src/acado_opt_control.cpp)
target_link_libraries(acado_opt_control ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${ACADO_SHARED_LIBRARIES})
add_dependencies(acado_opt_control ${catkin_EXPORTED_TARGETS})

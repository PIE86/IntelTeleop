cmake_minimum_required(VERSION 2.8.3)
project(intel_teleop_env)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(roscpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  intel_teleop_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

add_definitions(-std=c++11)



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
#   std_msgs  # Or other packages containing msgs
# )



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES intel_teleop_env
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(inlude
  ${roscpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


#add_library(minvolellipse lib/MinVolEllipse.cpp lib/MinVolEllipse_emxutil.cpp lib/norm.cpp lib/diag.cpp lib/inv.cpp lib/rt_nonfinite.cpp)
#target_link_libraries(minvolellipse ${roscpp_LIBRARIES} ${catkin_LIBRARIES})


## Declare a C++ executable
add_executable(environnement_generator src/environnement_generator.cpp)
target_link_libraries(environnement_generator ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(environnement_generator ${catkin_EXPORTED_TARGETS} ${roscpp_LIBRARIES})


cmake_minimum_required(VERSION 2.8.3)
project(sampler)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

add_service_files(
  FILES
  Sample.srv
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

add_definitions(-std=c++11)

include_directories(
  ${roscpp_INDLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(sampler_service src/sampler_service.cpp)
target_link_libraries(sampler_service ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(sampler_service ${catkin_EXPORTED_TARGETS})

cmake_minimum_required(VERSION 2.8.3)
project(intel_teleop_env)

find_package(roscpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  rospy
  std_msgs
  std_srvs
  intel_teleop_msgs
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

add_definitions(-std=c++11)


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

add_library(minvolellipse lib/MinVolEllipse.cpp lib/MinVolEllipse_emxutil.cpp lib/norm.cpp lib/diag.cpp lib/inv.cpp lib/rt_nonfinite.cpp)
target_link_libraries(minvolellipse ${roscpp_LIBRARIES} ${catkin_LIBRARIES})

add_library(ellipsoidgen src/ellipsoid_gen.cpp lib/rt_nonfinite.cpp lib/MinVolEllipse_terminate.cpp lib/MinVolEllipse_initialize.cpp lib/MinVolEllipse_emxAPI.cpp lib/rtGetNaN.cpp lib/rtGetInf.cpp)
target_link_libraries(ellipsoidgen ${roscpp_LIBRARIES} ${catkin_LIBRARIES} minvolellipse)



## Declare a C++ executable
add_executable(environment_generator src/environment_generator.cpp)
target_link_libraries(environment_generator ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ellipsoidgen)
add_dependencies(environment_generator ${catkin_EXPORTED_TARGETS} ${roscpp_LIBRARIES})




































cmake_minimum_required(VERSION 2.8.3)
project(intel_teleop_optcontrol)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        std_srvs
        intel_teleop_msgs
        )

FIND_PACKAGE(ACADO REQUIRED)

add_definitions(-std=c++11)

catkin_package()

###########
## Build ##
###########

include_directories(include
        ${roscpp_INDLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${ACADO_INCLUDE_DIRS}
        )

add_library(optcontrol src/optcontrol.cpp)
target_link_libraries(optcontrol ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${ACADO_SHARED_LIBRARIES})
add_dependencies(optcontrol intel_teleop_msgs_generate_messages_cpp)

add_executable(optimal_control src/optimal_control.cpp)
target_link_libraries(optimal_control ${catkin_LIBRARIES} ${roscpp_LIBRARIES} optcontrol)
add_dependencies(optimal_control ${catkin_EXPORTED_TARGETS})




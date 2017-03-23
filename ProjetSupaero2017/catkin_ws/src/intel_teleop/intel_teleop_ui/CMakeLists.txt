cmake_minimum_required(VERSION 2.8.3)
project(intel_teleop_ui)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        std_srvs
        intel_teleop_msgs
        )

catkin_package()

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(intel_teleop_user_interface src/user_interface.cpp)
add_dependencies(intel_teleop_user_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(intel_teleop_user_interface ${catkin_LIBRARIES})
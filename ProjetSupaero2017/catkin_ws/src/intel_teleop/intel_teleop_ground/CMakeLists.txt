cmake_minimum_required(VERSION 2.8.3)
project(intel_teleop_ground)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        std_srvs
        intel_teleop_msgs
        roslib
        )

add_definitions(-std=c++11)

catkin_package()

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
)

include_directories(
        ${roscpp_INDLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


add_executable(ground src/ground.cpp)
target_link_libraries(ground ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(ground intel_teleop_msgs_generate_messages_cpp)


install(TARGETS ground
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY Objects
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
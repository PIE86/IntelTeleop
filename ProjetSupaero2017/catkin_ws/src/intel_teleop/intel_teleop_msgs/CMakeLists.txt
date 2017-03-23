cmake_minimum_required(VERSION 2.8.3)
project(intel_teleop_msgs)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        std_srvs
        message_generation
        )

add_service_files(
        FILES
        addCylinderOptControl.srv
        addEllipseOptControl.srv
        enableMotors.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
)
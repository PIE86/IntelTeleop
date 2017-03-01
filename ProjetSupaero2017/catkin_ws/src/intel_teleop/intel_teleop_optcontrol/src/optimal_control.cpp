#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <intel_teleop_msgs/DroneState.h>
#include <intel_teleop_msgs/SpeedControl.h>
#include <intel_teleop_msgs/MotorControl.h>
#include <intel_teleop_msgs/UserInput.h>

void node_loop()
{
    ROS_INFO("%s", "optimal_control loop");
}


// Service pour ajouter cylindre

// Service pour ajouter cube (avec sphère == ellipse autour)

// Service pour ajouter sphere (avec ellipse autour)

// Callback du topic /clock pour déclencher la màj de l'opt-control.


int main(int argc, char **argv)
{
  ros::init(argc, argv, "optimal_control");
  ros::NodeHandle n;

  ros::Rate loop_rate(1);

  loop_rate.sleep();

  int count = 0;
  while (ros::ok())
  {
    node_loop();

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

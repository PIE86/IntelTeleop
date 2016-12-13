#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <intel_teleop_msgs/UserInput.h>

void node_loop()
{
    ROS_INFO("%s", "optimal_control loop");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "user_interface");
  ros::NodeHandle n;

  ros::Publisher user_input_topic = n.advertise<intel_teleop_msgs::UserInput>("user_input", 1000);

  
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

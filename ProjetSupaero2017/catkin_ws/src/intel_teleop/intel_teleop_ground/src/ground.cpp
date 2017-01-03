#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


void node_loop()
{
    ROS_INFO("%s", "ground loop");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground");
  ros::NodeHandle n;

  //ros::Publisher speed_control_topic = n.advertise<intel_teleop_msgs::SpeedControl>("speed_control", 1000);
  //ros::Subscriber encoder_feedback_topic = n.subscribe("estimated_state", 1000, estimated_state_feedback); 

  
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

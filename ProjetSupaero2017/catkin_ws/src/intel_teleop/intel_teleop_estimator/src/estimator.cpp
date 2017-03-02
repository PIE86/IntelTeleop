#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <intel_teleop_msgs/DroneState.h>

void real_state_feedback(const intel_teleop_msgs::DroneState state)
{
  ROS_INFO("Estimator heard real state");
}

void node_loop()
{
    ROS_INFO("%s", "estimator loop");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimator");
  ros::NodeHandle n;

  ros::Publisher estimated_state_topic = n.advertise<intel_teleop_msgs::DroneState>("estimated_state", 1000);
  ros::Subscriber real_state_topic = n.subscribe("real_state", 1000, real_state_feedback); 

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

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <intel_teleop_msgs/UserInput.h>
#include <intel_teleop_msgs/DroneState.h>
#include <intel_teleop_msgs/SpeedControl.h>

void estimated_state_feedback(const intel_teleop_msgs::DroneState state)
{
  //ROS_INFO("Ground heard drone state estimation");
}

void user_input_feedback(const intel_teleop_msgs::UserInput input)
{
  //ROS_INFO("Ground heard user input");
  ROS_INFO("User input: [%f,%f,%f] ; [%f,%f,%f]", 
		input.x, input.y, input.z, input.roll, input.pitch, input.yaw);
}

void node_loop()
{
    ROS_INFO("%s", "ground loop");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground");
  ros::NodeHandle n;
  
  ros::Publisher speed_control_topic = n.advertise<intel_teleop_msgs::SpeedControl>("speed_control", 1000);
  ros::Subscriber estimated_state_feedback_topic = n.subscribe("estimated_state", 1000, estimated_state_feedback);
  ros::Subscriber user_input_feedback_topic = n.subscribe("user_input", 1000, user_input_feedback); 

  
  ros::Rate loop_rate(10);

  loop_rate.sleep();

  int count = 0;
  while (ros::ok())
  {
    //node_loop();
    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

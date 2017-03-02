#include "ros/ros.h"
#include "std_msgs/String.h"

#include <intel_teleop_msgs/UserInput.h>
#include <intel_teleop_msgs/DroneState.h>
#include <intel_teleop_msgs/SpeedControl.h>

void estimated_state_feedback(const intel_teleop_msgs::DroneState state)
{
  //ROS_INFO("Ground heard drone state estimation");
  ROS_INFO("Drone state estimate: [x,y,z]=[%lf,%lf,%lf] ; [roll,pitch,yaw]=[%lf,%lf,%lf]", 
		state.x, state.y, state.z, state.roll, state.pitch, state.yaw);
}

void user_input_feedback(const intel_teleop_msgs::UserInput input)
{
  //ROS_INFO("Ground heard user input");
  ROS_INFO("User input: [x,y,z]=[%lf,%lf,%lf] ; [roll,pitch,yaw]=[%lf,%lf,%lf]", 
		input.x, input.y, input.z, input.roll, input.pitch, input.yaw);
}

void speed_control_feedback(const intel_teleop_msgs::SpeedControl speedCtrl)
{
  //ROS_INFO("Ground heard speed control");
  ROS_INFO("Speed control: [vx,vy,vz]=[%lf,%lf,%lf] ; [vroll,vpitch,vyaw]=[%lf,%lf,%lf]", 
		speedCtrl.v_x, speedCtrl.v_y, speedCtrl.v_z, speedCtrl.v_roll, speedCtrl.v_pitch, speedCtrl.v_yaw);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground");
  ros::NodeHandle n;
  
  ros::Subscriber estimated_state_feedback_topic = n.subscribe("estimated_state", 1000, estimated_state_feedback);
  ros::Subscriber user_input_feedback_topic = n.subscribe("user_input", 1000, user_input_feedback);
  ros::Subscriber speed_control_feedback_topic = n.subscribe("speed_control", 1000, speed_control_feedback);

  
  ros::Rate loop_rate(10);

  loop_rate.sleep();

  while (ros::ok())
  {    
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

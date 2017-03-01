#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "../lib/MinVolEllipse.h"

void node_loop()
{
    ROS_INFO("%s", "optimal_control loop");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ellipse_gen test");
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

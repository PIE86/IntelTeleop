#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "ellipsoid_gen.h"


void node_loop()
{
    ROS_INFO("%s", "environment generator loop ");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "environnement_generator");
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

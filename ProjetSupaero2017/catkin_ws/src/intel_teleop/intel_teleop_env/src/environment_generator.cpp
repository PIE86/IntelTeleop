#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "ellipsoid_gen.h"


using namespace std;

void node_loop()
{
    ROS_INFO("%s", "environment generator loop ");
    vector< vector<double> > points;
    points.push_back(vector<double>{1.0,1.0,1.0});
    points.push_back(vector<double>{1.0,3.0,3.0});
    points.push_back(vector<double>{3.0,1.0,1.0});
    points.push_back(vector<double>{3.0,3.0,3.0});

    Ellipsoid ellipse = GetMinVolEllipsoid(points, 0.1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "environment_generator");
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

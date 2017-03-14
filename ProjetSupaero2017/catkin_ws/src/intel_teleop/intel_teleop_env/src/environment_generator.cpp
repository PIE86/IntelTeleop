#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "ellipsoid_gen.h"


using namespace std;

void node_loop()
{
    ROS_INFO("%s", "environment generator loop ");
    vector<Point3D> points;
    points.push_back(Point3D(1.0,1.0,1.0));
    points.push_back(Point3D(1.0,3.0,3.0));
    points.push_back(Point3D(3.0,1.0,1.0));
    points.push_back(Point3D(3.0,3.0,3.0));

    bool res = GetMinVollEllipsoid(points, 0.1);
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

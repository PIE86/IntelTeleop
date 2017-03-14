#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include <string>


#include "intel_teleop_msgs/addCylinderOptControl.h"


std::string folderPath{};

bool addCylinder( intel_teleop_msgs::addCylinderOptControl::Request  &req,
                  intel_teleop_msgs::addCylinderOptControl::Response &answer)
{
  system( std::string( "rosrun xacro xacro " + folderPath + "/Objects/cylinder.xacro x:=5 y:=3 radius:=1 length:=5 > " + folderPath + "/object.urdf\n" ).c_str() );
  system( std::string( "rosrun gazebo_ros spawn_model -urdf -file " + folderPath + "/object.urdf -model model" ).c_str() );

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground");
  ros::NodeHandle n;

  auto serv = n.advertiseService("addCylinderOptControl", &addCylinder );

  folderPath = ros::package::getPath( "intel_teleop_ground" );

  ros::Rate loop_rate(10);
  loop_rate.sleep();

  while (ros::ok())
  {    
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

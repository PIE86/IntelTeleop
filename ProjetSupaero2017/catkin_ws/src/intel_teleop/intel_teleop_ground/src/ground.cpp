#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include <string>
#include <cmath>

#include "intel_teleop_msgs/addCylinderOptControl.h"

/// -------------------------
/// Functor to add a cylinder both in Gazebo and in the Optimal Control node.
/// -------------------------
class addCylinderCallback
{
	ros::NodeHandle *_n;
	std::string _folderPath;

public:
	addCylinderCallback( ros::NodeHandle *n, const std::string folderPath )
			: _n{ n },
			  _folderPath{ folderPath }
	{}

	bool operator()( intel_teleop_msgs::addCylinderOptControl::Request &req,
	                 intel_teleop_msgs::addCylinderOptControl::Response &answer )
	{
		// Computes the parameters of the cylinder for Gazebo.
		double length{ std::sqrt( std::pow( req.x1 - req.x2, 2. ) + std::pow( req.y1 - req.y2, 2. ) +
		                          std::pow( req.z1 - req.z2, 2. )) };
		double x{ ( req.x2 + req.x1 ) / 2. }, y{ ( req.y2 + req.y1 ) / 2. }, z{ ( req.z2 + req.z1 ) / 2. };
		double yaw{ std::asin(( req.x2 - req.x1 ) / length ) };
		double roll{ std::asin(( req.z2 - req.z1 ) / length ) + M_PI / 2. };

		// Uses Xacro to create the urdf model file.
		system( std::string( "rosrun xacro xacro " + _folderPath + "/Objects/cylinder.xacro " +
		                     "x:=" + std::to_string( x ) + " y:=" + std::to_string( y ) + " z:=" + std::to_string( z ) +
		                     " roll:=" + std::to_string( roll ) + " yaw:=" + std::to_string( yaw ) +
		                     " radius:=" + std::to_string( req.radius ) + " length:=" + std::to_string( length ) +
		                     " > " + _folderPath + "/object.urdf\n" ).c_str());

		// Adds the model.
		static int i{ 0 };
		system( std::string( "rosrun gazebo_ros spawn_model -urdf -file " + _folderPath + "/object.urdf -model model" +
		                     std::to_string( i++ )).c_str());
		system( std::string( "rm " + _folderPath + "/object.urdf" ).c_str());

		// Calls the services to add the cylinder to the optimal control.
		auto client = _n->serviceClient< intel_teleop_msgs::addCylinderOptControl >( "addCylinderOptControl" );
		intel_teleop_msgs::addCylinderOptControl srv{};
		srv.request = req;

		if( !client.call( srv ))
			return false;

		return true;
	}
};

int main( int argc, char **argv )
{
	ros::init( argc, argv, "ground" );
	ros::NodeHandle n{};

	std::string folderPath{ ros::package::getPath( "intel_teleop_ground" ) };

	auto serv = n.advertiseService< intel_teleop_msgs::addCylinderOptControl::Request,
			intel_teleop_msgs::addCylinderOptControl::Response >
			             ( "addCylinder", addCylinderCallback{ &n, folderPath } );


	ros::Rate loop_rate( 10 );
	while( ros::ok())
	{
		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}

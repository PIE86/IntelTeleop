#include "ros/ros.h"
#include "std_msgs/String.h"

#include "optcontrol.hpp"

#include <sstream>
#include <intel_teleop_msgs/enableMotors.h>

#include "hector_uav_msgs/MotorPWM.h"

/// -------------------------
/// Function to stop/start the motors
/// -------------------------
class EnableMotorsFunctor
{
	bool *_var;

public:
	EnableMotorsFunctor( bool *var )
	{ _var = var; }

	bool operator()( intel_teleop_msgs::enableMotors::Request &rq,
	                 intel_teleop_msgs::enableMotors::Response &answer )
	{
		*_var = rq.enable;

		if( *_var )
			ROS_INFO( "Motors enabled." );
		else
			ROS_INFO( "Motors disabled." );

		return true;
	}
};


int main( int argc, char **argv )
{
	ros::init( argc, argv, "optimal_control" );
	ros::NodeHandle n;

	bool enableMotors{ true };

	// Ponderations
	DMatrix Q( 12, 12 );
	Q( 0, 0 ) = Q( 1, 1 ) = 0.8; // Velocites along X and Y axis
	Q( 2, 2 ) = 10; // Velocity along Z axis
	Q( 3, 3 ) = Q( 4, 4 ) = Q( 5, 5 ) = Q( 6, 6 ) = 1e-6; // Motors PWM
	Q( 7, 7 ) = Q( 8, 8 ) = 1; // Phi and Theta: Attitude
	Q( 9, 9 ) = Q( 10, 10 ) = Q( 11, 11 ) = 0.3; // Angular velocities

	// Optimal Control during 0.6 second with steps lasting 0.1 second
	Optcontrol optControl{ Q, 0., 0.6, 0.1 };


	std::vector <ros::ServiceServer> servers;
	servers.push_back( n.advertiseService( "addCylinderOptControl", &Optcontrol::addCylinder, &optControl ));
	servers.push_back( n.advertiseService< intel_teleop_msgs::enableMotors::Request,
			intel_teleop_msgs::enableMotors::Response >
			                    ( "enableMotors", EnableMotorsFunctor( &enableMotors )));

	// IMU to receive angular velocities in the drone body frame
	auto imuSub = n.subscribe< sensor_msgs::Imu >( "/raw_imu", 1, &Optcontrol::setAngularVelocities, &optControl );

	// Estimated linear velocities and pose of the drone. Noisy.
//  auto poseSub = n.subscribe< geometry_msgs::PoseStamped >( "/pose", 1, &Optcontrol::setPose, &optControl );
//  auto velSub = n.subscribe< geometry_msgs::Vector3Stamped >( "/velocity", 1, &Optcontrol::setVelocities, &optControl );
	// Ground truth, no noise.
	auto groundTruthSub = n.subscribe< nav_msgs::Odometry >
			                       ( "/ground_truth/state", 1, &Optcontrol::setGroundTruth, &optControl );

	// Topic to receive the commands from the user interface
	auto cmdSub = n.subscribe< geometry_msgs::Twist >( "/command_velocity", 1, &Optcontrol::setRefVec,
	                                                   &optControl );

	// Topic to send the PWM commands to the drone.
	ros::Publisher motor_command = n.advertise< hector_uav_msgs::MotorPWM >( "/motor_pwm", 1 );


	ros::Rate loop_rate( 25 );
	loop_rate.sleep();

	std::vector< unsigned char > cmdVec( 4, 0 );
	int i{ 0 };

	while( ros::ok())
	{
		ROS_INFO( "%s", "optimal_control loop" );

		// Gets subscriptions.
		ros::spinOnce();

		// Periodically resets the control to avoid memory and cpu bloating
		if( ++i == 200 )
		{
			i = 0;
			optControl.reset();
		}

		auto cmd = optControl.solveOptimalControl();

		cmdVec[ 0 ] = static_cast< unsigned char >( cmd( 0 ));
		cmdVec[ 1 ] = static_cast< unsigned char >( cmd( 1 ));
		cmdVec[ 2 ] = static_cast< unsigned char >( cmd( 2 ));
		cmdVec[ 3 ] = static_cast< unsigned char >( cmd( 3 ));

		if( !enableMotors )
			cmdVec[ 3 ] = cmdVec[ 2 ] = cmdVec[ 1 ] = cmdVec[ 0 ] = 0;

		hector_uav_msgs::MotorPWM cmdMsg;
		cmdMsg.pwm = cmdVec;
		motor_command.publish( cmdMsg );

		loop_rate.sleep();
	}

	return 0;
}

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ctime>
#include <cmath>

#include "optcontrol.hpp"

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>


Optcontrol::Optcontrol( DMatrix &Q, const double t_in, const double t_fin, const double dt )
		: _xEst{ 12 }
{
	// Create the Optimal Control Process running from t_in (0s) to t_fin(0.6s for us), with steps of dt(0.1s for us)
	_ocp = std::unique_ptr< OCP >( new OCP( t_in, t_fin, static_cast< int >(( t_fin - t_in ) / dt )));


	// --- Introducing constants used by the model.
	// Some are taken from the model of the drone.
	// Others are based on some model's values but modified to improve stability and better match our simplified system.
	const double c = 0.000025; // Electrical torque of the motors. EMPIRICAL
	const double Cf = 0.000246; // Factor between rotational speed of the motor and the propulsion force. EMPIRICAL
	const double d = 0.275; // Distance from the center to the propeller. TAKEN FROM MODEL
	const double Jx = 0.01152; // Inertia around X-axis. TAKEN FROM MODEL
	const double Jy = 0.01152; // Inertia around Y-axis. TAKEN FROM MODEL
	const double Jz = 0.0218; // Inertia around Z-axis. TAKEN FROM MODEL
	const double m = 1.477; // Mass. TAKEN FROM MODEL
	const double g = 9.81; // Gravity.

	// --- Introducing the states used by the model and given by the simulation.
	// x, y, z : position
	// vx, vy, vz : linear velocity
	// Psi, Theta, Phi : orientation (Yaw-Pitch-Roll = Euler(3,2,1))
	// p, q, r : angular velocity
	DifferentialState x, y, z, vx, vy, vz, psi, theta, phi, p, q, r;

	// --- Introducing the control states used by the model and sent to the simulated drone.
	// u1, u2, u3, u4 : input pwm of the propellers
	Control u1, u2, u3, u4;

	// -------------------------
	// SIMPLIFIED MODEL OF A DRONE
	// -------------------------
	{
		// EQUATIONS

		_f << dot( x ) == vx;
		_f << dot( y ) == vy;
		_f << dot( z ) == vz;

		// Speed along each axis is determined by the total propulsion force and the drone's orientation.
		_f << dot( vx ) == Cf * ( u1 * u1 + u2 * u2 + u3 * u3 + u4 * u4 ) *
		                   ( cos( psi ) * sin( theta ) * cos( phi ) + sin( psi ) * sin( phi )) / m;
		_f << dot( vy ) == Cf * ( u1 * u1 + u2 * u2 + u3 * u3 + u4 * u4 ) *
		                   ( sin( psi ) * sin( theta ) * cos( phi ) - cos( psi ) * sin( phi )) / m;
		_f << dot( vz ) == Cf * ( u1 * u1 + u2 * u2 + u3 * u3 + u4 * u4 ) *
		                   cos( phi ) * cos( theta ) / m - g;

		_f << dot( psi ) == sin( phi ) / cos( theta ) * q + cos( phi ) / cos( theta ) * r;
		_f << dot( theta ) == cos( phi ) * q - sin( phi ) * r;
		_f << dot( phi ) == p + sin( phi ) * tan( theta ) * q + cos( phi ) * tan( theta ) * r;

		// Drone looks like:
		//       1
		//       |
		//    4--x--2
		//       |
		//       3
		_f << dot( p ) == ( d * Cf * ( u4 * u4 - u2 * u2 ) + ( Jy - Jz ) * q * r ) / Jx;
		_f << dot( q ) == -( d * Cf * ( u1 * u1 - u3 * u3 ) + ( Jx - Jz ) * p * r ) / Jy;
		_f << dot( r ) == -( c * ( -u1 * u1 + u2 * u2 - u3 * u3 + u4 * u4 ) + ( Jx - Jy ) * p * q ) / Jz;

		_ocp->subjectTo( _f );
	}
	{
		// CONSTRAINTS

		// Constraints on the velocity of each propeller to avoid extreme behaviours
		_ocp->subjectTo( 25 <= u1 <= 225 );
		_ocp->subjectTo( 25 <= u2 <= 225 );
		_ocp->subjectTo( 25 <= u3 <= 225 );
		_ocp->subjectTo( 25 <= u4 <= 225 );

		// Constraint to avoid singularity
		_ocp->subjectTo( -1.25 <= theta <= 1.25 );
		// Constraint to avoid flipping over.
		_ocp->subjectTo( -1.25 <= phi <= 1.25 );
	}
	// -------------------------

	// -------------------------
	// COST FUNCTION
	// -------------------------
	{
		// Speeds references for the drone
		_refVec = DVector{ 12 };
		// Optimal pwm for the propellers: stationnary flight
		_refVec[ 3 ] = _refVec[ 4 ] = _refVec[ 5 ] = _refVec[ 6 ] = 122;

		// Cost function, Least Square type
		// The OCP tries to minimize (LSQ) the difference between the values and the references from above (modified by the
		// user's inputs)
		_h << vx << vy << vz;
		_h << u1 << u2 << u3 << u4;
		_h << theta << phi;
		_h << p << q << r;

		// Weights matrix
		if( _Q.getNumCols() != 12 || _Q.getNumRows() != 12 )
			ROS_FATAL( "Weight matrix size is not suitable for this model" );
		_Q = Q;

		_ocp->minimizeLSQ( _Q, _h, _refVec );
	}

	// SET UP THE MPC CONTROLLER:
	// --------------------------
	_alg = std::unique_ptr< RealTimeAlgorithm >( new RealTimeAlgorithm( *_ocp ));
	_alg->set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
	_alg->set( INTEGRATOR_TYPE, INT_RK45 );
	_alg->set( MAX_NUM_ITERATIONS, 3 );
	_alg->set( PRINT_COPYRIGHT, false );
	_alg->set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );

	_controller = std::unique_ptr< Controller >( new Controller( *_alg ));
	_controller->init( 0., _xEst );

	// Starts the clock used by the controller
	_previousClock = ros::Time::now();
	_t = 0.;
};

bool Optcontrol::addCylinder( intel_teleop_msgs::addCylinderOptControl::Request &cyl,
                              intel_teleop_msgs::addCylinderOptControl::Response &answer )
{
	// If the control is not started, we cannot add obstacles
	if( _t < -0.5 )
		return false;

	_cylinders.push_back( Cylinder{ cyl.radius, cyl.x1, cyl.y1, cyl.z1, cyl.x2, cyl.y2, cyl.z2 } );

	return true;
}

//double Optcontrol::distCyl(const Cylinder &cyl, double x, double y, double z)
//{
//  double length{ std::sqrt( std::pow( cyl.c1.x - cyl.c2.x, 2. ) +
//                            std::pow( cyl.c1.y - cyl.c2.y, 2. ) +
//                            std::pow( cyl.c1.z - cyl.c2.z, 2. ) ) };
//
//  std::array< double, 3 > V{ ( cyl.c2.x - cyl.c1.x ) / length, ( cyl.c2.y - cyl.c1.y ) / length, ( cyl.c2.z - cyl.c1.z ) / length };
//
//  double distOrtho{ std::sqrt( std::pow( ( x - cyl.c1.x ) * V[ 1 ] - ( y - cyl.c1.y ) * V[ 0 ], 2. ) +
//                               std::pow( ( y - cyl.c1.y ) * V[ 2 ] - ( z - cyl.c1.z ) * V[ 1 ], 2. ) +
//                               std::pow( ( z - cyl.c1.z ) * V[ 0 ] - ( x - cyl.c1.x ) * V[ 2 ], 2. ) ) };
//
//  double k{ ( V[ 0 ] * ( x - cyl.c1.x ) + V[ 1 ] * ( y - cyl.c1.y ) + V[ 2 ] * ( z - cyl.c1.z ) ) /
//            ( std::pow( V[ 0 ], 2.0) + std::pow( V[ 1 ], 2.0) + std::pow( V[ 2 ], 2.0) ) };
//
//  if( k < 0.0 )
//    return std::sqrt( std::pow( k, 2. ) + std::pow( distOrtho, 2. ) );
//  else if( k > length )
//    return std::sqrt( std::pow( k - length, 2. ) + std::pow( distOrtho, 2. ) );
//  else
//    return distOrtho;
//}


DVector Optcontrol::avoidance( DVector &localRef )
{
	// Avoid only vertical cylinders.
	for( int i{ 0 }; i < _cylinders.size(); i++ )
	{
		double dist{ std::sqrt(
				std::pow( _xEst[ 0 ] - _cylinders[ i ].c1.x, 2. ) + std::pow( _xEst[ 1 ] - _cylinders[ i ].c1.y, 2. )) };
		double angle{ std::atan2( _xEst[ 1 ] - _cylinders[ i ].c1.y, _xEst[ 0 ] - _cylinders[ i ].c1.x ) };
		double speed{ std::sqrt( std::pow( localRef[ 0 ], 2. ) + std::pow( localRef[ 1 ], 2. )) };

		// 0.75 is the approximate radius of the drone.
		if( dist < _cylinders[ i ].radius + 0.75 )
		{
			ROS_ERROR( "Collision" );
		}
			// Push the drone away by giving it a command opposed to its current speed vector
		else if( dist < _cylinders[ i ].radius * 3. + 0.75 )
		{
			localRef[ 0 ] += ( 2. + speed ) * cos( angle );
			localRef[ 1 ] += ( 2. + speed ) * sin( angle );
		}
			// Push less when the drone is further
		else if( dist < _cylinders[ i ].radius * 5. + 0.75 )
		{
			localRef[ 0 ] += ( 5. - dist ) / 2.5 * ( 2. + speed ) * cos( angle );
			localRef[ 1 ] += ( 5. - dist ) / 2.5 * ( 2. + speed ) * sin( angle );
		}
	}

	return localRef;
}

DVector Optcontrol::solveOptimalControl()
{
	// If the control is not started, send 0 to avoid early takeoff
	if( _t < -0.5 )
		return DVector{ 4 };

	// Modify the orders to avoid the obstacles
	auto localRef = avoidance( _refVec );

	// Add the new user's orders to the control algorithm
	VariablesGrid referenceVG( localRef, Grid{ _t, _t + 0.6, static_cast< int >( 0.6 / 0.1 ) } );
	referenceVG.setVector( 0, _lastRefVec );
	_alg->setReference( referenceVG );
	_lastRefVec = localRef;

// Controller step
	if( _controller->step( _t, _xEst ) != 0 )
	{
		ROS_ERROR( "controller failed " );
		return 1;
	}

	// Get the control for this step
	DVector u( 4 );
	_controller->getU( u );

	// Update the internal clock.
	auto currentClock{ ros::Time::now() };
	double dt{ ( currentClock - _previousClock ).toSec() };
	_previousClock = currentClock;
	_t += dt;

	return u;
}

DMatrix Optcontrol::getMatrixQ()
{
	return _Q;
}

void Optcontrol::setMatrixQ( DMatrix &Q )
{
	_Q = Q;
}

DVector Optcontrol::getrefVec()
{
	return _refVec;
}

void Optcontrol::setrefVec( DVector &refVec )
{
	_refVec = refVec;
}


void Optcontrol::setAngularVelocities( const sensor_msgs::Imu::ConstPtr &imu )
{
	_xEst[ 9 ] = imu->angular_velocity.x;
	_xEst[ 10 ] = imu->angular_velocity.y;
	_xEst[ 11 ] = imu->angular_velocity.z;
}

void Optcontrol::setPose( const geometry_msgs::PoseStamped::ConstPtr &pose )
{
	// Position
	_xEst[ 0 ] = pose->pose.position.x;
	_xEst[ 1 ] = pose->pose.position.y;
	_xEst[ 2 ] = pose->pose.position.z;

	// Orientation
	double x{ pose->pose.orientation.x };
	double y{ pose->pose.orientation.y };
	double z{ pose->pose.orientation.z };
	double w{ pose->pose.orientation.w };

	// roll (x-axis rotation)
	double t0{ 2. * ( w * x + y * z ) };
	double t1{ 1. - 2. * ( std::pow( x, 2. ) + std::pow( y, 2. )) };
	_xEst[ 8 ] = std::atan2( t0, t1 );

	// pitch (y-axis rotation)
	double t2{ 2. * ( w * y - z * x ) };
	t2 = t2 > 1. ? 1. : t2;
	t2 = t2 < -1. ? -1. : t2;
	_xEst[ 7 ] = std::asin( t2 );

	// yaw (z-axis rotation)
	double t3{ 2. * ( w * z + x * y ) };
	double t4{ 1. - 2. * ( std::pow( y, 2. ) + std::pow( z, 2. )) };
	_xEst[ 6 ] = std::atan2( t3, t4 );
}

void Optcontrol::setVelocities( const geometry_msgs::Vector3Stamped::ConstPtr &vel )
{
	// Linear velocities
	_xEst[ 3 ] = vel->vector.x;
	_xEst[ 4 ] = vel->vector.y;
	_xEst[ 5 ] = vel->vector.z;
}

void Optcontrol::setRefVec( const geometry_msgs::Twist::ConstPtr &refVec )
{
	// Update the user's inputs
	_refVec[ 0 ] = refVec->linear.x;
	_refVec[ 1 ] = refVec->linear.y;
	_refVec[ 2 ] = refVec->linear.z;
	_refVec[ 11 ] = refVec->angular.z;
}

void Optcontrol::setGroundTruth( const nav_msgs::Odometry::ConstPtr &groundTruth )
{
	// Position
	_xEst[ 0 ] = groundTruth->pose.pose.position.x;
	_xEst[ 1 ] = groundTruth->pose.pose.position.y;
	_xEst[ 2 ] = groundTruth->pose.pose.position.z;

	// Linear velocities
	_xEst[ 3 ] = groundTruth->twist.twist.linear.x;
	_xEst[ 4 ] = groundTruth->twist.twist.linear.y;
	_xEst[ 5 ] = groundTruth->twist.twist.linear.z;

	// Orientation
	double x{ groundTruth->pose.pose.orientation.x };
	double y{ groundTruth->pose.pose.orientation.y };
	double z{ groundTruth->pose.pose.orientation.z };
	double w{ groundTruth->pose.pose.orientation.w };

	// roll (x-axis rotation)
	double t0{ 2. * ( w * x + y * z ) };
	double t1{ 1. - 2. * ( std::pow( x, 2. ) + std::pow( y, 2. )) };
	_xEst[ 8 ] = std::atan2( t0, t1 );

	// pitch (y-axis rotation)
	double t2{ 2. * ( w * y - z * x ) };
	t2 = t2 > 1. ? 1. : t2;
	t2 = t2 < -1. ? -1. : t2;
	_xEst[ 7 ] = std::asin( t2 );

	// yaw (z-axis rotation)
	double t3{ 2. * ( w * z + x * y ) };
	double t4{ 1. - 2. * ( std::pow( y, 2. ) + std::pow( z, 2. )) };
	_xEst[ 6 ] = std::atan2( t3, t4 );
}

void Optcontrol::reset()
{
	if( _t > 0.5 )
		_controller->init( _t, _xEst );
}
#include <memory>

#include <ros/ros.h>

#include <acado_toolkit.hpp>

#include "intel_teleop_msgs/addCylinderOptControl.h"
//#include "intel_teleop_msgs/addEllipseOptControl.h"

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

USING_NAMESPACE_ACADO


class Optcontrol
{
private:
	// Optimal Control Problem and equations
	std::unique_ptr <OCP> _ocp;
	DifferentialEquation _f;

	// Cost function and weights matrix
	Function _h;
	DMatrix _Q;
	std::unique_ptr <RealTimeAlgorithm> _alg;

	std::unique_ptr <Controller> _controller;

	// Estimated state received via topics
	DVector _xEst;
	// Orders
	DVector _refVec, _lastRefVec;

	// Internal clock
	double _t;
	ros::Time _previousClock;


	struct Point3D
	{
		double x, y, z;

		Point3D( double x, double y, double z )
				: x{ x },
				  y{ y },
				  z{ z }
		{}
	};

	struct Cylinder
	{
		double radius;
		Point3D c1, c2;

		Cylinder( double radius, double x1, double y1, double z1, double x2, double y2, double z2 )
				: radius{ radius },
				  c1{ x1, y1, z1 },
				  c2{ x2, y2, z2 }
		{}
	};

	std::vector <Cylinder> _cylinders;

//	double distCyl( const Cylinder &cyl, double x, double y, double z );

	/// -------------------------
	/// Modify the orders to avoid obstacles
	/// In:  - LocalRef: orders
	/// Out: - LocalRef: modified orders
	/// -------------------------
	DVector avoidance( DVector &localRef );


public:

	/// -------------------------
	/// In: - Q: Weights matrix for the optimal control
	///     - t_in: Time of start of the optimal control (0s)
	///     - t_fin: Time of end of the optimal control (0.6s for us)
	///     - dt: Time of a control step (0.1s for us)
	/// -------------------------
	Optcontrol( DMatrix &Q, const double t_in, const double t_fin, const double dt );


	/// -------------------------
	/// Adds a cylinder to avoid
	/// In:     - cyl: Cylinder to add
	/// In-Out: - answer: Nothing.
	/// Out: True if the service adds the cylinder
	/// -------------------------
	bool addCylinder( intel_teleop_msgs::addCylinderOptControl::Request &cyl,
	                  intel_teleop_msgs::addCylinderOptControl::Response &answer );

//    bool addEllipse( intel_teleop_msgs::addEllipseOptControl::Request &req,
//                     intel_teleop_msgs::addEllipseOptControl::Response &ans );


	/// -------------------------
	/// Runs a step of the optimal control problem
	/// Out: Computed PWM controls
	/// -------------------------
	DVector solveOptimalControl();


	/// -------------------------
	/// Sets the internal estimated states from the /pose topic
	/// In: - pose: Pose of the drone computed by the Hector model
	/// -------------------------
	void setPose( const geometry_msgs::PoseStamped::ConstPtr &pose );

	/// -------------------------
	/// Sets the internal estimated states from the /velocity topic
	/// In: - vel: Velocities of the drone computed by the Hector model
	/// -------------------------
	void setVelocities( const geometry_msgs::Vector3Stamped::ConstPtr &vel );

	/// -------------------------
	/// Sets the internal estimated states from the /imu topic
	/// In: - imu: IMU state of the drone computed by the Hector model (drone body frame)
	/// -------------------------
	void setAngularVelocities( const sensor_msgs::Imu::ConstPtr &imu );

	/// -------------------------
	/// Sets the internal estimated states from the /ground_truth/state topic
	/// In: - groundTruth: Global state of the drone computed by the Hector model (gazebo frame)
	/// -------------------------
	void setGroundTruth( const nav_msgs::Odometry::ConstPtr &groundTruth );

	/// -------------------------
	/// Changes the orders of the drone
	/// In: - refVec: New orders from the /command_velocity topic
	/// -------------------------
	void setRefVec( const geometry_msgs::Twist::ConstPtr &refVec );


	/// -------------------------
	/// Resets the optimal control
	/// -------------------------
	void reset();
};

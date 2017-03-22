#include <memory>

#include <ros/ros.h>

#include "model.hpp"

#include "intel_teleop_msgs/addCylinderOptControl.h"
#include "intel_teleop_msgs/addEllipseOptControl.h"
#include "intel_teleop_msgs/startOptControl.h"

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


class Optcontrol {

private:
    DMatrix _Q;
    Function _h;
    std::unique_ptr< Controller > _controller;
    std::unique_ptr< RealTimeAlgorithm > _alg;
    std::unique_ptr< Process > _process;
    DifferentialEquation _f;
    std::unique_ptr< OCP > _ocp;

    DVector _xEst;
    DVector _refVec, _lastRefVec;

    // Phi: roll, Theta: pitch, Psi: yaw, p = wX, q = wY, r = wZ
    DifferentialState x, y, z;

    bool _started;

    double _t;
    ros::Time _previousClock;

    DVector _currentState;

    struct Point3D
    {
        double x, y, z;
        Point3D( double x, double y, double z ) : x{ x }, y{ y }, z{ z } {}
    };
    struct Cylinder
    {
        double radius;
        Point3D c1, c2;
        Cylinder( double radius, double x1, double y1, double z1, double x2, double y2, double z2 )
            : radius{ radius }, c1{ x1, y1, z1 }, c2{ x2, y2, z2 }
        {}
    };

    std::vector< Cylinder > _cylinders;

    double distCyl( const Cylinder& cyl, double x, double y, double z );

    DVector avoidance( DVector& localRef );


public:

    Optcontrol(DMatrix &Q, const double t_in, const double t_fin, const double dt);


    void init( DMatrix &Q, const double t_in, const double t_fin, const double dt );

    bool addCylinder( intel_teleop_msgs::addCylinderOptControl::Request &cyl,
                      intel_teleop_msgs::addCylinderOptControl::Response &answer );

//    bool addEllipse( intel_teleop_msgs::addEllipseOptControl::Request &req,
//                     intel_teleop_msgs::addEllipseOptControl::Response &ans );

    DVector solveOptimalControl( );


    void setPose( const geometry_msgs::PoseStamped::ConstPtr &pose );

    void setVelocities( const geometry_msgs::Vector3Stamped::ConstPtr &vel );

    void setAngularVelocities( const sensor_msgs::Imu::ConstPtr &imu );

    void setRefVec( const geometry_msgs::Twist::ConstPtr &refVec );

    void setGroundTruth( const nav_msgs::Odometry::ConstPtr &groundTruth );

    DMatrix getMatrixQ();

    void setMatrixQ(DMatrix &Q);

    DVector getrefVec();

    void setrefVec(DVector &refVec);

    void reset();
};

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "optcontrol.hpp"

#include <sstream>
#include <intel_teleop_msgs/DroneState.h>
#include <intel_teleop_msgs/SpeedControl.h>
#include <intel_teleop_msgs/MotorControl.h>
#include <intel_teleop_msgs/UserInput.h>


// Service pour ajouter cylindre

// Service pour ajouter cube (avec sphère == ellipse autour)

// Service pour ajouter sphere (avec ellipse autour)

// Callback du topic /clock pour déclencher la màj de l'opt-control.


int main(int argc, char **argv) {
  ros::init(argc, argv, "optimal_control");
  ros::NodeHandle n;

  // Ponderations
  DMatrix Q(10,10);
  Q(0,0) = Q(1,1) = Q(2,2) = 1e-1;
  Q(3,3) = Q(4,4) = Q(5,5) = Q(6,6) = 1e-9;
  Q(7,7) = Q(8,8) = Q(9,9) = 1e-1;

  // Cmd ?
  DVector refVec{ 10 };
  refVec.setZero( 10 );

  // Pose ?
  DVector X_0{ 12 };
  X_0.setZero();
  X_0( 2 ) = 4.;

  Optcontrol optControl{ Q, refVec, 0., 1., 0.25, X_0 };

  // Advertises the services used by the simulation.
  std::vector< ros::ServiceServer > servers;

  servers.push_back( n.advertiseService("addCylinderOptControl", &Optcontrol::addCylinder, &optControl ) );
  servers.push_back( n.advertiseService("addEllipseOptControl", &Optcontrol::addEllipse, &optControl ) );
  servers.push_back( n.advertiseService("startOptControl", &Optcontrol::completeSimulation, &optControl ) );

  // Ajouter un topic pour récupérer la clock de gazebo.

  ros::Rate loop_rate(1);

  while( ros::ok() )
  {
    ROS_INFO("%s", "optimal_control loop");

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "optcontrol.hpp"

#include <sstream>
#include <intel_teleop_msgs/DroneState.h>
#include <intel_teleop_msgs/SpeedControl.h>
#include <intel_teleop_msgs/MotorControl.h>
#include <intel_teleop_msgs/UserInput.h>

#include "hector_uav_msgs/MotorPWM.h"


// Service pour ajouter cylindre

// Service pour ajouter cube (avec sphère == ellipse autour)

// Service pour ajouter sphere (avec ellipse autour)

// Callback du topic /clock pour déclencher la màj de l'opt-control.


int main(int argc, char **argv) {
  ros::init(argc, argv, "optimal_control");
  ros::NodeHandle n;

  // Ponderations
  DMatrix Q(12,12);
  Q(0,0) = Q(1,1) = 5;
  Q(2,2) = 10;
  Q(3,3) = Q(4,4) = Q(5,5) = Q(6,6) = 1e-6;
  Q(7,7) = Q(8,8) = 1;
  Q(9,9) = Q(10,10) = Q(11,11) = 0.1; //0.05
  //Q(10,10) = Q(11,11) = 1e-1;

  // Cmd ?
  Optcontrol optControl{ Q, 0., 0.04, 0.04 };

  // Advertises the services used by the simulation.
  std::vector< ros::ServiceServer > servers;

  servers.push_back( n.advertiseService("addCylinderOptControl", &Optcontrol::addCylinder, &optControl ) );
  servers.push_back( n.advertiseService("addEllipseOptControl", &Optcontrol::addEllipse, &optControl ) );
  servers.push_back( n.advertiseService("startOptControl", &Optcontrol::completeSimulation, &optControl ) );

  // Ajouter un topic pour récupérer la clock de gazebo.
  // /clock -> publication tick gazebo pour connaître le t entre deux pas de simulation.
  // msg.clock, type time
  // PAS BESOIN, il suffit de mettre /use_sim_time à true

  // Ajouter un topic pour récupérer l'état du drone (voir NoteS).
  // /imu -> angles et vitesses angulaires. /velocity, vitesses linéaires. /pose (pose_estimator), positions linéaires.
  // imu : msg.angular_velocity.x, y, z
  // velocity : msg.vector.x, y, z
  // pose : msg.position.x, y, z
  // pose : msg.orientation.x, y, z, w (convertir)
  // AJOUTER hector_quadrotor_pose_estimator nrstiuers
  auto imuSub = n.subscribe< sensor_msgs::Imu >( "/raw_imu", 1, &Optcontrol::setAngularVelocities, &optControl );
//  auto poseSub = n.subscribe< geometry_msgs::PoseStamped >( "/pose", 1, &Optcontrol::setPose, &optControl );
//  auto velSub = n.subscribe< geometry_msgs::Vector3Stamped >( "/velocity", 1, &Optcontrol::setVelocities,
//                                                                         &optControl );

  auto groundTruthSub = n.subscribe< nav_msgs::Odometry >( "/ground_truth/state", 1, &Optcontrol::setGroundTruth, &optControl );

  // Ajouter un topic pour récupérer les commandes clavier (/cmd_vel, faut rediriger..., voir Bertrand).
  auto cmdSub = n.subscribe< geometry_msgs::Twist >( "/command_velocity", 1, &Optcontrol::setRefVec,
                                                              &optControl );

  // Ajouter un topic pour envoyer les commandes de vol.
  ros::Publisher motor_command = n.advertise< hector_uav_msgs::MotorPWM >( "/motor_pwm", 1 );

  sleep( 5 );


  ros::Rate loop_rate( 50 );

  std::vector< unsigned char > cmdVec( 4, 0 );

  int i{ 0 };


  while( ros::ok() )
  {
    ROS_INFO("%s", "optimal_control loop");

    // Gets subscriptions.
    ros::spinOnce();

    auto cmd = optControl.solveOptimalControl();
    if( ++i == 200 )
    {
      i = 0;
      optControl.reset();
    }

    hector_uav_msgs::MotorPWM cmdMsg;
    cmdVec[ 0 ] = static_cast< unsigned char >( cmd( 0 ) );//* 0 + 90;
    cmdVec[ 1 ] = static_cast< unsigned char >( cmd( 1 ) );// * 0 + 90;
    cmdVec[ 2 ] = static_cast< unsigned char >( cmd( 2 ) );// * 0 + 90;
    cmdVec[ 3 ] = static_cast< unsigned char >( cmd( 3 ) );// * 0 + 90;

    if( cmdVec[ 0 ] == 0 )
      cmdVec[ 1 ] = cmdVec[ 3 ] = cmdVec[ 0 ] = cmdVec[ 2 ] = 0;

    cmdMsg.pwm = cmdVec;
    motor_command.publish( cmdMsg ); // Publish msg

    loop_rate.sleep();
  }

  return 0;
}

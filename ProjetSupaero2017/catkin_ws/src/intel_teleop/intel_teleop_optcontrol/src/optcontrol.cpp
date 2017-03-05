#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ctime>

#include "optcontrol.hpp"

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>


Optcontrol::Optcontrol(DMatrix &Q, const double t_in, const double t_fin, const double dt, bool isPWD)
    : _t{ -1. }, _xEst{ 12 }
{
  init(Q, t_in, t_fin, dt, isPWD);
};

bool Optcontrol::addCylinder(intel_teleop_msgs::addCylinderOptControl::Request &c,
                             intel_teleop_msgs::addCylinderOptControl::Response &answer)
{
  if ( _t > -0.5 )
    return false;

  _ocp->subjectTo(pow(c.radius + 1, 2) <=
                  (pow((y - c.y1) * (c.z2 - c.z1) - (c.y2 - c.y1) * (z - c.z1), 2) +
                   pow((z - c.z1) * (c.x2 - c.x1) - (c.z2 - c.z1) * (x - c.x1), 2) +
                   pow((x - c.x1) * (c.y2 - c.y1) - (c.x2 - c.x1) * (y - c.y1), 2)) /
                  (pow(c.x2 - c.x1, 2) + pow(c.y2 - c.y1, 2) + pow(c.z2 - c.z1, 2)));

  return true;
}

bool Optcontrol::addEllipse(intel_teleop_msgs::addEllipseOptControl::Request &req,
                            intel_teleop_msgs::addEllipseOptControl::Response &ans)
{
  if ( _t > -0.5 )
    return false;
  // Quid ?
  return false;
}

void Optcontrol::init(DMatrix &Q, DVector &refVec, const double t_in, const double t_fin, const double dt, bool isPWD)
{

  _Q = Q;
  _refVec = DVector{ 10 };

  // Introducing constants
  const double c = 0.00001;
  const double Cf = 0.00018;
  const double d = 0.250;
  const double Jx = 0.01152;
  const double Jy = 0.01152;
  const double Jz = 0.0218;
  const double m = 1.477;
  const double g = 9.81;

  _ocp = std::unique_ptr<OCP>(new OCP(t_in, t_fin, static_cast< int >((t_fin - t_in) / dt )));


  if (isPWD)
  {
    DifferentialState vx, vy, vz, phi, theta, psi, p, q, r;

    // x, y, z : position
    // vx, vy, vz : linear velocity
    // phi, theta, psi : orientation (Yaw-Pitch-Roll = Euler(3,2,1))
    // Roll pitch yaw plutôt, compte tenu des formules et du rapport... ? http://adg.stanford.edu/aa208/dynamics/notation.html
    // p, q, r : angular velocity
    // u1, u2, u3, u4 : velocity of the propellers
    Control u1, u2, u3, u4;


    _f << dot(x) == vx;
    _f << dot(y) == vy;
    _f << dot(z) == vz;
    _f << dot(vx) ==
    -Cf * (u1 * u1 + u2 * u2 + u3 * u3 + u4 * u4) * (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)) / m;
    _f << dot(vy) ==
    -Cf * (u1 * u1 + u2 * u2 + u3 * u3 + u4 * u4) * (sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)) / m;
    _f << dot(vz) ==
    Cf * (u1 * u1 + u2 * u2 + u3 * u3 + u4 * u4) * cos(psi) * cos(theta) / m - g; // axe z vers le bas - NON !
    _f << dot(phi) == p + sin(phi) * tan(theta) * q + cos(phi) * tan(theta) * r;
    _f << dot(theta) == cos(phi) * q - sin(phi) * r;
    _f << dot(psi) == sin(phi) / cos(theta) * q + cos(phi) / cos(theta) * r;
    _f << dot(p) == (d * Cf * (u4 * u4 - u2 * u2) + (Jy - Jz) * q * r) / Jx;
    _f << dot(q) == (d * Cf * (u1 * u1 - u3 * u3) + (Jz - Jx) * p * r) / Jy;
    _f << dot(r) == (c * (-u1 * u1 + u2 * u2 - u3 * u3 + u4 * u4) + (Jx - Jy) * p * q) / Jz;




    // Constraints on the velocity of each propeller
    _ocp->subjectTo(16 <= u1 <= 255);
    _ocp->subjectTo(16 <= u2 <= 255);
    _ocp->subjectTo(16 <= u3 <= 255);
    _ocp->subjectTo(16 <= u4 <= 255);

    // Constraint to avoid singularity
    _ocp->subjectTo(-1. <= theta <= 1.);

    _h << vx << vy << vz;
    _h << u1 << u2 << u3 << u4;
    _h << p << q << r;

    if (_Q.getNumCols() != 10 || _Q.getNumRows() != 10)
    {
      std::cout << "Weight matrix size is not suitable for this model" << std::endl;
    }


  } else
  {

    DifferentialState phi, theta, psi;

    // x, y, z : position
    // vx, vy, vz : linear velocity
    // phi, theta, psi : orientation (Yaw-Pitch-Roll = Euler(3,2,1))

    Control u_vx, u_vy, u_vz, u_p, u_q, u_r; // Linear and angular velocity

    _f << dot(x) == u_vx;
    _f << dot(y) == u_vy;
    _f << dot(z) == u_vz;
    _f << dot(phi) == u_p + sin(phi) * tan(theta) * u_q + cos(phi) * tan(theta) * u_r;
    _f << dot(theta) == cos(phi) * u_q - sin(phi) * u_r;
    _f << dot(psi) == sin(phi) / cos(theta) * u_q + cos(phi) / cos(theta) * u_r;
    // Quid ? Pas de variable ax, ay, az ?
//        _f << ax == dot(u_vx);
//        _f << ay == dot(u_vy);
//        _f << az == dot(u_vz);

    _ocp->subjectTo(-15 <= u_p <= 15);
    _ocp->subjectTo(-15 <= u_q <= 15);
    _ocp->subjectTo(-15 <= u_r <= 15);

    _h << u_vx << u_vy << u_vz;
    _h << u_p << u_q << u_r;

    if (_Q.getNumCols() != 6 || _Q.getNumRows() != 6)
    {
      std::cout << "Weight matrix size is not suitable for this model" << std::endl;
    }

  }

  _ocp->minimizeLSQ(_Q, _h, _refVec);
  _ocp->subjectTo(_f);
}

bool Optcontrol::completeSimulation(intel_teleop_msgs::startOptControl::Request &req,
                                    intel_teleop_msgs::startOptControl::Response &ans)
{
  DynamicSystem dynamicSystem(_f, OutputFcn{});
  _process = std::unique_ptr<Process>(new Process(dynamicSystem, INT_RK45));


  // SET UP THE MPC CONTROLLER:
  // --------------------------
  _alg = std::unique_ptr<RealTimeAlgorithm>(new RealTimeAlgorithm(*_ocp));
  _alg->set(INTEGRATOR_TYPE, INT_RK45);
  _alg->set(MAX_NUM_ITERATIONS, 1);
  _alg->set(PRINT_COPYRIGHT, false);
  _alg->set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);

  _controller = std::unique_ptr<Controller>(new Controller(*_alg));

  // SETTING UP THE SIMULATION ENVIRONMENT:
  // --------------------------------------
  DVector u(4);
  u.setZero();
  _controller->init(0., _xEst);
  _process->init(0., _xEst, u);

  _previousClock = ros::Time::now();
  _t = 0.;

  return true;
}


DVector Optcontrol::solveOptimalControl()
{
  if( _t < -0.5 )
    return DVector{ 4 };

 /* if (abs(NewRefVec[0] - _refVec(0)) > 1.)
  {
    if (NewRefVec[0] > _refVec(0))
    {
      NewRefVec[0] = _refVec(0) + 1.;
    } else
    {
      NewRefVec[0] = _refVec(0) - 1.;
    }
  }

  if (abs(NewRefVec[1] - _refVec(1)) > 1.)
  {
    if (NewRefVec[1] > _refVec(1))
    {
      NewRefVec[1] = _refVec(1) + 1.;
    } else
    {
      NewRefVec[1] = _refVec(1) - 1.;
    }
  }

  if (abs(NewRefVec[2] - _refVec(2)) > 1.)
  {
    if (NewRefVec[2] > _refVec(2))
    {
      NewRefVec[2] = _refVec(2) + 1.;
    } else
    {
      NewRefVec[2] = _refVec(2) - 1.;
    }
  }*/

  //double refT[10] = {NewRefVec[0], NewRefVec[1], NewRefVec[2], 0., 0., 0., 0., 0., 0., 0.};
//  double refT[10] = {0., 0., 1., 0., 0., 0., 0., 0., 0., 0.};
//  DVector refVecN(10, refT);
  VariablesGrid referenceVG(_refVec, Grid{_t, _t + 1., 4});
  referenceVG.setVector(0, _refVec);
  _alg->setReference(referenceVG);
//  setrefVec(refVecN);


  // Inclure état réel drone.

/*
DVector X;
VariablesGrid Y;
Y.setZero();

// get state vector
process.getY(Y);
X = Y.getLastVector();
*/

// MPC step
// compute the command
  ROS_INFO( "x: %f, y:%f, z:%f", _xEst[ 0 ], _xEst[ 1 ], _xEst[ 2 ] );
  bool success = _controller->step(_t, _xEst);

  if (success != 0)
  {
    std::cout << "controller failed " <<
              std::endl;
    return 1;
  }
  DVector u(4);
  u.init();

  u.setZero();

  _controller->getU(u);


  auto currentClock{ ros::Time::now() };
  double dt{ ( currentClock - _previousClock ).toSec() };
  _previousClock = currentClock;
  _process->step(_t, _t + dt, u);
  _t += dt;

  return u;
}

DMatrix Optcontrol::getMatrixQ()
{
  return _Q;
}

void Optcontrol::setMatrixQ(DMatrix &Q)
{
  _Q = Q;
}

DVector Optcontrol::getrefVec()
{
  return _refVec;
}

void Optcontrol::setrefVec(DVector &refVec)
{
  _refVec = refVec;
}


void Optcontrol::setAngularVelocities(const sensor_msgs::Imu::ConstPtr &imu)
{
  // imu : msg.angular_velocity.x, y, z

  _xEst[ 9 ] = imu->angular_velocity.x;
  _xEst[ 10 ] = imu->angular_velocity.y;
  _xEst[ 11 ] = imu->angular_velocity.z;
}

void Optcontrol::setPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
  // pose : msg.position.x, y, z
  // pose : msg.orientation.x, y, z, w (convertir)

  _xEst[ 0 ] = pose->pose.position.x;
  _xEst[ 1 ] = pose->pose.position.y;
  _xEst[ 2 ] = pose->pose.position.z;


  double x{ pose->pose.orientation.x };
  double y{ pose->pose.orientation.y };
  double z{ pose->pose.orientation.z };
  double w{ pose->pose.orientation.w };

  // roll (x-axis rotation)
  double t0{ 2. * ( w * x + y * z) };
  double t1{ 1. - 2. * ( std::pow( x, 2. ) + std::pow( y, 2. ) ) };
  _xEst[ 6 ] = std::atan2( t0, t1 );

  // pitch (y-axis rotation)
  double t2{ 2. * ( w * y - z * x ) };
  t2 = t2 > 1. ? 1. : t2;
  t2 = t2 < -1. ? -1. : t2;
  _xEst[ 7 ] = std::asin( t2 );

  // yaw (z-axis rotation)
  double t3{ 2. * ( w * z + x * y ) };
  double t4{ 1. - 2. * ( std::pow( y, 2. ) + std::pow( z, 2. ) ) };
  _xEst[ 8 ] = std::atan2( t3, t4 );
}

void Optcontrol::setVelocities(const geometry_msgs::Vector3Stamped::ConstPtr &vel)
{
  // velocity : msg.vector.x, y, z

  _xEst[ 3 ] = vel->vector.x;
  _xEst[ 4 ] = vel->vector.y;
  _xEst[ 5 ] = vel->vector.z;
}

void Optcontrol::setRefVec(const geometry_msgs::Twist &refVec)
{
  _refVec[ 0 ] = refVec->linear.x;
  _refVec[ 1 ] = refVec->linear.y;
  _refVec[ 2 ] = refVec->linear.z;
  _refVec[ 7 ] = refVec->linear.x;
  _refVec[ 8 ] = refVec->linear.x;
  _refVec[ 9 ] = refVec->linear.x;
}
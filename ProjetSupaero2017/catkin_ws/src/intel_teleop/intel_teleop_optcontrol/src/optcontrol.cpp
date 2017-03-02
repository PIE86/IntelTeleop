#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ctime>

#include "optcontrol.hpp"

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>


Optcontrol::Optcontrol(DMatrix &Q, DVector &refVec,
                       const double t_in, const double t_fin, const double dt, DVector &X_0, bool isPWD)
    : _X_0{X_0}, _started{false}
{
  init(Q, refVec, t_in, t_fin, dt, isPWD);
};

bool Optcontrol::addCylinder(intel_teleop_msgs::addCylinderOptControl::Request &c,
                             intel_teleop_msgs::addCylinderOptControl::Response &answer)
{
  if (_started)
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
  if (_started)
    return false;
  // Quid ?
  return false;
}

void Optcontrol::init(DMatrix &Q, DVector &refVec, const double t_in, const double t_fin, const double dt, bool isPWD)
{

  _Q = Q;
  _refVec = refVec;



  // Introducing constants
  const double c = 0.00001;
  const double Cf = 0.00065;
  const double d = 0.250;
  const double Jx = 0.018;
  const double Jy = 0.018;
  const double Jz = 0.026;
  const double m = 0.9;
  const double g = 9.81;

  _ocp = std::unique_ptr<OCP>(new OCP(t_in, t_fin, static_cast< int >((t_fin - t_in) / dt )));


  if (isPWD)
  {
    DifferentialState vx, vy, vz, phi, theta, psi, p, q, r;

    // x, y, z : position
    // vx, vy, vz : linear velocity
    // phi, theta, psi : orientation (Yaw-Pitch-Roll = Euler(3,2,1))
    // p, q, r : angular velocity
    // u1, u2, u3, u4 : velocity of the propellers
    Control u1, u2, u3, u4;


    _f << dot(x) == vx;
    _f << dot(y) == vy;
    _f << dot(z) == vz;
    _f << dot(phi) == p + sin(phi) * tan(theta) * q + cos(phi) * tan(theta) * r;
    _f << dot(theta) == cos(phi) * q - sin(phi) * r;
    _f << dot(psi) == sin(phi) / cos(theta) * q + cos(phi) / cos(theta) * r;
    _f << dot(p) == (d * Cf * (u4 * u4 - u2 * u2) + (Jy - Jz) * q * r) / Jx;
    _f << dot(q) == (d * Cf * (u1 * u1 - u3 * u3) + (Jz - Jx) * p * r) / Jy;
    _f << dot(r) == (c * (-u1 * u1 + u2 * u2 - u3 * u3 + u4 * u4) + (Jx - Jy) * p * q) / Jz;
    _f << dot(vx) ==
    -Cf * (u1 * u1 + u2 * u2 + u3 * u3 + u4 * u4) * (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)) / m;
    _f << dot(vy) ==
    -Cf * (u1 * u1 + u2 * u2 + u3 * u3 + u4 * u4) * (sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)) / m;
    _f << dot(vz) ==
    -Cf * (u1 * u1 + u2 * u2 + u3 * u3 + u4 * u4) * cos(psi) * cos(theta) / m + g; // axe z vers le bas



    // Constraints on the velocity of each propeller
    _ocp->subjectTo(16 <= u1 <= 95);
    _ocp->subjectTo(16 <= u2 <= 95);
    _ocp->subjectTo(16 <= u3 <= 95);
    _ocp->subjectTo(16 <= u4 <= 95);

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
  _controller->init(0., _X_0);
  _process->init(0., _X_0, u);

  _started = true;
  return true;
}


DVector Optcontrol::solveOptimalControl(DVector &NewRefVec, DVector &x_est, double &t)
{

  if (abs(NewRefVec[0] - _refVec(0)) > 1.)
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
  }

  double refT[10] = {NewRefVec[0], NewRefVec[1], NewRefVec[2], 0., 0., 0., 0., 0., 0., 0.};
  DVector refVecN(10, refT);
  VariablesGrid referenceVG(refVecN, Grid{t, t + 1., 2});
  referenceVG.setVector(0, _refVec);
  _alg->setReference(referenceVG);
  setrefVec(refVecN);


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
  bool success = _controller->step(t, x_est);

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


  std::clock_t t_new = std::clock();
  double dt = (double) (t_new - t) / (double) CLOCKS_PER_SEC;
  _process->step(t, t + dt, u);
  t += dt;

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


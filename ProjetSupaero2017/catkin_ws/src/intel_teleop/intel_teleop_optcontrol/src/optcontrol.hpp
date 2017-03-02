#include <memory>

#include "model.hpp"

#include "intel_teleop_msgs/addCylinderOptControl.h"
#include "intel_teleop_msgs/addEllipseOptControl.h"
#include "intel_teleop_msgs/startOptControl.h"


class Optcontrol {

private:
    DMatrix _Q;
    Function _h;
    DVector _refVec;
    std::unique_ptr< Controller > _controller;
    std::unique_ptr< RealTimeAlgorithm > _alg;
    std::unique_ptr< Process > _process;
    DifferentialEquation _f;
    std::unique_ptr< OCP > _ocp;
    DVector& _X_0;

    DifferentialState x, y, z;

    bool _started;

public:

    Optcontrol(DMatrix &Q, DVector &refVec,
               const double t_in, const double t_fin, const double dt, DVector &X_0, bool isPWD = true);


    void init( DMatrix &Q, DVector &refVec, const double t_in, const double t_fin, const double dt, bool isPWD );

    bool addCylinder( intel_teleop_msgs::addCylinderOptControl::Request &c,
                      intel_teleop_msgs::addCylinderOptControl::Response &answer );

    bool addEllipse( intel_teleop_msgs::addEllipseOptControl::Request &req,
                     intel_teleop_msgs::addEllipseOptControl::Response &ans );

    bool completeSimulation( intel_teleop_msgs::startOptControl::Request &req,
                             intel_teleop_msgs::startOptControl::Response &ans );


    DVector solveOptimalControl(DVector &NewRefVec, DVector &x_est, double &t );

    DMatrix getMatrixQ();

    void setMatrixQ(DMatrix &Q);

    DVector getrefVec();

    void setrefVec(DVector &refVec);


};

#include <iomanip>
#include <iostream>

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include "opt_control/OptControl.h"


bool solve(opt_control::OptControl::Request &req,
           opt_control::OptControl::Response &res)
{
    USING_NAMESPACE_ACADO

    DifferentialState s,v,m; // the differential states
    Control u; // the control input u
    Parameter T; // the time horizon T
    DifferentialEquation f(0.0, T); // the differential equation

    OCP ocp(0.0, T); // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm(T); // the time T should be optimized

    f << dot(s) == v; // an implementation
    f << dot(v) == (u - 0.2 * v * v)/m; // of the model equations
    f << dot(m) == -0.01 * u * u; // for the rocket.

    ocp.subjectTo(f); // minimize T s.t. the model,
    ocp.subjectTo(AT_START, s == 0.0); // the initial values for s,
    ocp.subjectTo(AT_START, v == 0.0); // v,
    ocp.subjectTo(AT_START, m == 1.0); // and m,

    ocp.subjectTo(AT_END, s == 10.0); // the terminal constraints for s
    ocp.subjectTo(AT_END, v ==  0.0); // and v,

    ocp.subjectTo(-0.1 <= v <= 1.7); // as well as the bounds on v
    ocp.subjectTo(-1.1 <= u <= 1.1); // the control input u,
    ocp.subjectTo(5.0 <= T <= 15.0); // and the time horizon T.

    OptimizationAlgorithm algorithm(ocp); // construct optimization algorithm,
    algorithm.solve(); // and solve the problem.

    VariablesGrid states, parameters, controls;

    algorithm.getDifferentialStates(states);
    algorithm.getParameters(parameters);
    algorithm.getControls(controls);

    // Print each tuple [time, value] of control
    // getVector(t) return the control values as a vector
    for(unsigned t = 0; t < controls.getNumPoints(); t++)
      std::cout << std::setprecision(8) << controls.getTime(t) << ", "
        << controls.getVector(t)[0] << std::endl;
    std::cout << std::endl;

    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "solve_rocket_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("solve_rocket", solve);
  ROS_INFO("Ready to do Rocket Science.");
  ros::spin();

  return 0;
}

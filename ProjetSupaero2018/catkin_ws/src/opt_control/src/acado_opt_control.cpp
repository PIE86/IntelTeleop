#include <iomanip>
#include <iostream>
#include <vector>

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

#include "ros/ros.h"

#include "opt_control/OptControl.h"

#define ACADO_VERBOSE true
#define PLOT true

// TODO: check succes

USING_NAMESPACE_ACADO

// default nb_controls
const unsigned int DEFAULT_NB_CONTROLS = 20; // path length -> 21

// state vector norm threshold above which 2 states are considered differents
const double THRESHOLD = 0.1;
double TMIN = 0.0;

void log_results(VariablesGrid states, VariablesGrid controls, VariablesGrid parameters);

OptimizationAlgorithm create_algorithm_car(
  std::vector<double> s1, std::vector<double> s2, int nb_controls,
  std::vector<double> init_states,
  std::vector<double> init_controls, double init_cost,
  int NX, int NU);

bool check_success(
  int returnValue, std::vector<double> s1,
  std::vector<double> s2, VariablesGrid states, VariablesGrid controls,
  double T, double threshold);



bool solve(opt_control::OptControl::Request &req,
           opt_control::OptControl::Response &res)
{
  std::vector<double> s1 = req.s1;
  std::vector<double> s2 = req.s2;
  // Decides which will be the size of trajectories
  int nb_controls;
  if (req.states.size() == 0){
    nb_controls = DEFAULT_NB_CONTROLS;
  }
  else{
    nb_controls = req.states.size()/req.NX;
  }

  // cf https://sourceforge.net/p/acado/discussion/general/thread/6e434f88/
  clearAllStaticCounters();

  // std::cout << "s1:" << s1 << '\n';
  // std::cout << "s2:" << s2 << '\n';

  auto toto = req.states;
  auto titi = req.controls;

  OptimizationAlgorithm algorithm = create_algorithm_car(s1, s2, nb_controls, toto, titi, req.cost, req.NX, req.NU);

  // solve the problem, returnValue is a status describing how the calcul went
  // HACK: logs produced during optimization dumped into a file
  if (!ACADO_VERBOSE){
    std::cout.setstate(std::ios_base::failbit);
  }
  int returnValue = algorithm.solve();
  if (!ACADO_VERBOSE){
    std::cout.clear();
  }

  VariablesGrid states, parameters, controls;
  algorithm.getDifferentialStates(states);
  algorithm.getParameters(parameters);
  algorithm.getControls(controls);

  // log_results(states, controls, parameters);

  // Convert into ROS data structures
  const int nb_points = controls.getNumPoints();
  std::vector<double> time_arr(nb_points);
  std::vector<double> states_arr(nb_points*req.NX);
  std::vector<double> controls_arr(nb_points*req.NU);

  // getVector(t) return the control values as a vector
  for (unsigned i = 0; i < nb_points; i++)
  {
    time_arr[i] = states.getTime(i);
    for (unsigned j = 0; j < req.NX; j++){
      states_arr[i+j] = states.getVector(i)[j];
    }
    for (unsigned j = 0; j < req.NU; j++){
      controls_arr[i+j] = controls.getVector(i)[j];
    }
  }

  res.states = states_arr;
  res.controls = controls_arr;
  res.time = parameters.getVector(0)[0];
  res.success = check_success(returnValue, s1, s2, states, controls, res.time, THRESHOLD);

  return true;
}


OptimizationAlgorithm create_algorithm_car(
    std::vector<double> s1, std::vector<double> s2, int nb_controls,
    std::vector<double> init_states,
    std::vector<double> init_controls, double init_cost,
    int NX, int NU){
    
  // Wheel parameters
	const double r = 0.2; // radius
	const double m = 2; // mass
	const double J = 0.5 * m * r*r; // inertia  
    
  // Wheel model
  DifferentialState x, y, theta, v;  
  Control c, w;  // c is the torque w the yaw rate
  IntermediateState a = c * r/J; // acceleration
  
  Parameter T;
  DifferentialEquation f(0.0, T); // the differential equation

  OCP ocp(0.0, T, nb_controls); // time horizon of the OCP: [0,T], , number of control points
  ocp.minimizeMayerTerm(T); // the time T should be optimized
  
	// DifferentialEquation
  f << dot(x) == v * cos(theta);
  f << dot(y) == v * sin(theta);
  f << dot(theta) == w;
  f << dot(v) ==  a;
  
  ocp.subjectTo(f);
  ocp.subjectTo(AT_START, x == s1[0]);
  ocp.subjectTo(AT_START, y == s1[1]);
  ocp.subjectTo(AT_START, theta == s1[2]);
  ocp.subjectTo(AT_START, v == s1[3]);

  ocp.subjectTo(AT_END, x == s2[0]);
  ocp.subjectTo(AT_END, y == s2[1]);
  ocp.subjectTo(AT_END, theta == s2[2]);
  ocp.subjectTo(AT_END, v == s2[3]);

  // TODO: bounds
	ocp.subjectTo( -15 <= a <= 10); // braking capability > acceleration capability
	ocp.subjectTo( -M_PI <= w <= M_PI);
  ocp.subjectTo(TMIN <= T); // and the time horizon T.

  OptimizationAlgorithm algorithm(ocp);

  algorithm.set( MAX_NUM_ITERATIONS, 80 );

  // ----------------------------------
  // INITIALIZATION
  // ----------------------------------

  if (init_controls.size() > 0){
    Grid timeGrid( 0.0, 1.0, nb_controls);
    VariablesGrid x_init(NX, timeGrid);
    VariablesGrid u_init(NU, timeGrid);
    VariablesGrid p_init(1, timeGrid);
    
    for (unsigned i = 0; i < nb_controls; i++){
      for (unsigned j = 0; j < 3; j++){
        x_init(i, j) = init_states[i+j];
        u_init(i, j) = init_controls[i+j];
      }
    }
	    
	algorithm.initializeDifferentialStates(x_init);
  algorithm.initializeControls(u_init);
  algorithm.initializeParameters(p_init);
  }

  if (PLOT){
    GnuplotWindow window;
    window.addSubplot(x, "DifferentialState x");
    window.addSubplot(y, "DifferentialState y");
    window.addSubplot(theta, "DifferentialState theta");
    window.addSubplot(v, "DifferentialState v");
    window.addSubplot(a, "IntermediateState a");
    window.addSubplot(w, "Control w");
    algorithm << window;
  }

  return algorithm;
}



bool check_success(
  int returnValue, std::vector<double> s1,
  std::vector<double> s2, VariablesGrid states, VariablesGrid controls,
  double T, double threshold){
  // TODO
  if (returnValue == RET_OPTALG_SOLVE_FAILED){
    return false;
  }
  if (returnValue == RET_MAX_NUMBER_OF_STEPS_EXCEEDED){
    std::cout << "\n\n\n\n\n\n" << '\n';
    std::cout << "MAX ITERATIONS REACHED" << '\n'; // Never seen ?
    return false;
  }
  if (T < TMIN){
    return false;
  }
  // - else if dist between start/end and s1 s2 > THRESHOLD -> false
  return true;
}


void log_results(VariablesGrid states, VariablesGrid controls, VariablesGrid parameters){
  const int nb_points = controls.getNumPoints();
  std::cout << std::setprecision(5) << "Results" << "\n";
  std::cout << '\n' << "Number of points:" << nb_points << std::endl;
  for(unsigned i = 0; i < controls.getNumPoints(); i++){
    std::cout << std::setprecision(5)
    << states.getTime(i) << ", "
    << "parameters: " << parameters.getVector(i)[0] << ", "
    << "sx: " << states.getVector(i)[0] << ", "
    << "sy: " << states.getVector(i)[1] << ", "
    << "sz: " << states.getVector(i)[2] << ", "
    << "ux: " << controls.getVector(i)[0]
    << std::endl;
  }
  std::cout << std::endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "solve_ocp_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("solve_ocp", solve);
  ROS_INFO("Ready to solve OCP.");
  ros::spin();

  return 0;
}

#include <iomanip>
#include <iostream>
#include <vector>

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include "opt_control/OptControl.h"

// TODO: check succes

USING_NAMESPACE_ACADO

// default nb_controls
const unsigned int DEFAULT_NB_CONTROLS = 21;
// state vector norm threshold above which 2 states are considered differents
const float THRESHOLD = 0.1;
float TMIN = 5.0;

void log_results(VariablesGrid states, VariablesGrid controls, VariablesGrid parameters);
std::tuple<std::vector<geometry_msgs::Point>, std::vector<geometry_msgs::Point>> get_point_lsts(VariablesGrid states, VariablesGrid controls);
OptimizationAlgorithm create_algorithm_rocket(geometry_msgs::Point p1, geometry_msgs::Point p2, int nb_controls,
                                              std::vector<geometry_msgs::Point> init_states, std::vector<geometry_msgs::Point> init_controls, float cost);
bool check_success(geometry_msgs::Point p1, geometry_msgs::Point p2, VariablesGrid states, VariablesGrid controls, float T, float threshold);


bool solve(opt_control::OptControl::Request &req,
           opt_control::OptControl::Response &res)
{
  geometry_msgs::Point p1 = req.p1;
  geometry_msgs::Point p2 = req.p2;
  int nb_controls;
  if (req.states.size() == 0){
    nb_controls = DEFAULT_NB_CONTROLS;
  }
  else{
    nb_controls = req.states.size();
  }

  std::cout << "P1:" << p1 << '\n';
  std::cout << "P2:" << p2 << '\n';

  OptimizationAlgorithm algorithm = create_algorithm_rocket(p1, p2, nb_controls, req.states, req.controls, req.cost);
  algorithm.solve(); // solve the problem.
  VariablesGrid states, parameters, controls;
  algorithm.getDifferentialStates(states);
  algorithm.getParameters(parameters);
  algorithm.getControls(controls);

  // log_results(states, controls, parameters);

  // Convert into ROS data structures
  auto states_controls = get_point_lsts(states, controls);
  res.states = std::get<0>(states_controls);
  res.controls = std::get<1>(states_controls);
  res.time = parameters.getVector(0)[0];
  res.success = check_success(p1, p2, states, controls, res.time, THRESHOLD);

  return true;
}

OptimizationAlgorithm create_algorithm_rocket(geometry_msgs::Point p1, geometry_msgs::Point p2, int nb_controls,
                                              std::vector<geometry_msgs::Point> init_states, std::vector<geometry_msgs::Point> init_controls, float init_cost){
  DifferentialState s,v,m; // the differential states
  Control u; // the control input u
  Parameter T; // the time horizon T
  DifferentialEquation f(0.0, T); // the differential equation

  OCP ocp(0.0, T, nb_controls); // time horizon of the OCP: [0,T], , number of control points
  ocp.minimizeMayerTerm(T); // the time T should be optimized

  f << dot(s) == v; // an implementation
  f << dot(v) == (u - 0.2 * v * v)/m; // of the model equations
  f << dot(m) == -0.01 * u * u; // for the rocket.

  ocp.subjectTo(f); // minimize T s.t. the model,
  ocp.subjectTo(AT_START, s == p1.x); // the initial values for s,
  ocp.subjectTo(AT_START, v == p1.y); // v,
  ocp.subjectTo(AT_START, m == p1.z); // and m,

  ocp.subjectTo(AT_END, s == p2.x); // the terminal constraints for s
  ocp.subjectTo(AT_END, v == p2.y); // and v,

  ocp.subjectTo(-0.1 <= v <= 1.7); // as well as the bounds on v
  ocp.subjectTo(-1.1 <= u <= 1.1); // the control input u,
  ocp.subjectTo(TMIN <= T <= 15.0); // and the time horizon T.

  OptimizationAlgorithm algorithm(ocp);

  // ----------------------------------
  // INITIALIZATION
  // ----------------------------------
  if (init_controls.size() > 0){
    Grid timeGrid( 0.0, 1.0, 11 );
    VariablesGrid x_init(3, timeGrid);
    VariablesGrid u_init(1, timeGrid);
    VariablesGrid p_init(1, timeGrid);

    for (unsigned i = 0; i < nb_controls; i++){
      for (unsigned j = 0; j < 3; j++){
        if (j == 0){
          x_init(i, j) = init_states[i].x;
          u_init(i, j) = init_controls[i].x;
        }
        if (j == 1){
          x_init(i, j) = init_states[i].y;
        }
        if (j == 2){
          x_init(i, j) = init_states[i].z;
        }
      }
    }
    p_init(0, 0) = init_cost;

    algorithm.initializeDifferentialStates(x_init);
    algorithm.initializeControls(u_init);
    algorithm.initializeParameters(p_init);
  }

  return algorithm;
}


bool check_success(geometry_msgs::Point p1, geometry_msgs::Point p2, VariablesGrid states, VariablesGrid controls, float T, float threshold){
  // TODO
  bool succes = false;
  if (T < TMIN){
    succes = false;
  }
  // - else if dist between start/end and p1 p2 > THRESHOLD -> false
  return succes;
}


std::tuple<std::vector<geometry_msgs::Point>, std::vector<geometry_msgs::Point>> get_point_lsts(VariablesGrid states, VariablesGrid controls){
  const int nb_points = controls.getNumPoints();
  std::vector<float> time_arr(nb_points);
  std::vector<geometry_msgs::Point> states_arr(nb_points);
  std::vector<geometry_msgs::Point> controls_arr(nb_points);
  geometry_msgs::Point s_point;
  geometry_msgs::Point u_point;

  // getVector(t) return the control values as a vector
  for(unsigned i = 0; i < controls.getNumPoints(); i++)
  {
    time_arr[i] = states.getTime(i);
    s_point.x = states.getVector(i)[0];
    s_point.y = states.getVector(i)[1];
    s_point.z = states.getVector(i)[2];
    u_point.x = controls.getVector(i)[0];
    states_arr[i] = s_point;
    controls_arr[i] = u_point;
  }

  return std::make_tuple(states_arr, controls_arr);
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
  ros::init(argc, argv, "solve_rocket_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("solve_rocket", solve);
  ROS_INFO("Ready to do Rocket Science.");
  ros::spin();

  return 0;
}

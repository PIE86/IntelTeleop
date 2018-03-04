#include <iomanip>
#include <iostream>
#include <vector>

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

#include <actionlib/server/simple_action_server.h>

#include "ros/ros.h"

#include "roadmap/OptControlAction.h"


#include "modelConstants.h"

#define ACADO_VERBOSE false
#define PLOT false
#define OBSTACLES true

USING_NAMESPACE_ACADO

// default nb_controls
const unsigned int DEFAULT_NB_CONTROLS = 21; // path length -> 21
// state vector norm threshold above which 2 states are considered differents
const double THRESHOLD = 0.1;




typedef actionlib::SimpleActionServer<roadmap::OptControlAction> Server;

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



bool solve(const roadmap::OptControlGoalConstPtr& goal, Server* as)
{
  // TODO
  std::vector<double> s1 = goal->s1;
  std::vector<double> s2 = goal->s2;
  // Decides which will be the size of trajectories
  int nb_controls;
  if (goal->states.size() == 0){
    nb_controls = DEFAULT_NB_CONTROLS;
  }
  else{
    nb_controls = goal->states.size()/goal->NX;
  }

  // cf https://sourceforge.net/p/acado/discussion/general/thread/6e434f88/
  clearAllStaticCounters();

  // std::cout << "s1:" << s1 << '\n';
  // std::cout << "s2:" << s2 << '\n';

  // TODO: proper renaming
  auto toto = goal->states;
  auto titi = goal->controls;

  OptimizationAlgorithm algorithm = create_algorithm_car(s1, s2, nb_controls, toto, titi, goal->cost, goal->NX, goal->NU);

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
  std::vector<double> states_arr(nb_points*goal->NX);
  std::vector<double> controls_arr(nb_points*goal->NU);

  // getVector(t) return the control values as a vector
  for (unsigned i = 0; i < nb_points; i++)
  {
    time_arr[i] = states.getTime(i);
    for (unsigned j = 0; j < goal->NX; j++){
      states_arr[goal->NX*i+j] = states.getVector(i)[j];
    }
    for (unsigned j = 0; j < goal->NU; j++){
      controls_arr[goal->NU*i+j] = controls.getVector(i)[j];
    }
  }

  roadmap::OptControlResult result;

  result.states = states_arr;
  result.controls = controls_arr;
  result.time = parameters.getVector(0)[0];
  result.success = check_success(returnValue, s1, s2, states, controls, result.time, THRESHOLD);

  as->setSucceeded(result, "coucou");

  return true;
}


OptimizationAlgorithm create_algorithm_car(
    std::vector<double> s1, std::vector<double> s2, int nb_controls,
    std::vector<double> init_states,
    std::vector<double> init_controls, double init_cost,
    int NX, int NU){
  // Wheel model
  // (x, y) is the position of the wheel in the world and theta its angle
  DifferentialState x, y, theta;
  // v is the velocity and w the angle speed
  Control v, w;
  Parameter T;
  DifferentialEquation f(0.0, T); // the differential equation

  OCP ocp(0.0, T, nb_controls); // time horizon of the OCP: [0,T], , number of control points
  ocp.minimizeMayerTerm(T); // the time T should be optimized

  f << dot(x) == v * cos(theta);
  f << dot(y) == v * sin(theta);
  f << dot(theta) == w;

  ocp.subjectTo(f);

  // Set constraints
  ocp.subjectTo(AT_START, x == s1[0]);
  ocp.subjectTo(AT_START, y == s1[1]);
  ocp.subjectTo(AT_START, theta == s1[2]);

  ocp.subjectTo(AT_END, x == s2[0]);
  ocp.subjectTo(AT_END, y == s2[1]);
  ocp.subjectTo(AT_END, theta == s2[2]);

  ocp.subjectTo(X_MIN <= x <= X_MAX);
  ocp.subjectTo(Y_MIN <= y <= Y_MAX);

  ocp.subjectTo(V_MIN <= v <= V_MAX);
  ocp.subjectTo(W_MIN <= w <= W_MAX);
  ocp.subjectTo(TMIN <= T); // and the time horizon T.

  // The car has to avoid obstacles
  if (OBSTACLES){
    std::vector<double> obstacles;
    if (ros::param::get("/obstacles/obstacles_vec", obstacles)){
      for(unsigned int obstacle_index = 0; obstacle_index < obstacles.size()/3; obstacle_index++){
        double obs_x = obstacles.at(3 * obstacle_index);
        double obs_y = obstacles.at(3 * obstacle_index + 1);
        double obs_r = obstacles.at(3 * obstacle_index + 2);

        ocp.subjectTo(pow(x - obs_x, 2) + pow(y - obs_y, 2) >= pow(obs_r, 2));
      }
    } else {
      ROS_INFO("\033[1;31mCould not read obstacles... Please run the obstacles/read_obstacles_server node before !!! \033[0m");
    }
  }

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
        x_init(i, j) = init_states[NX*i+j];
        u_init(i, j) = init_controls[NU*i+j];
      }
    }
    p_init(0, 0) = init_cost;

    algorithm.initializeDifferentialStates(x_init);
    algorithm.initializeControls(u_init);
    algorithm.initializeParameters(p_init);
  }

  if (PLOT){
    GnuplotWindow window;
    window.addSubplot(x, "DifferentialState x");
    window.addSubplot(y, "DifferentialState y");
    window.addSubplot(theta, "DifferentialState theta");
    window.addSubplot(v, "Control v");
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

  ros::init(argc, argv, "do_dishes_server");
  ros::NodeHandle n;
  Server server(n, "solve_ocp", boost::bind(&solve, _1, &server), false);
  server.start();

  ros::service::waitForService("read_obstacles", 5000);
  ROS_INFO("Ready to solve OCP.");
  ros::spin();

  return 0;
}

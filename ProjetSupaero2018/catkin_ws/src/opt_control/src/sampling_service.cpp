#include <random>
#include <math.h>
#include <vector>

#include "ros/ros.h"

#include "obstacles/CheckPoint.h"

#include "opt_control/Samples.h"

#include "modelConstants.h"

#define CHECK_OBSTACLES true
#define VERBOSE true


bool checkValidity(float x, float y){
  if (!CHECK_OBSTACLES){
    // bypass the call to the
    return true;
  }
  ros::NodeHandle n;
  ros::ServiceClient client = \
    n.serviceClient<obstacles::CheckPoint>("check_point");
  obstacles::CheckPoint srv;
  srv.request.x = x;
  srv.request.y = y;

  if (client.call(srv)) {
    if (VERBOSE){
      if (!srv.response.is_valid){
        std::cout << x << " " << y << " not valid" << '\n';
      }
    }
    return srv.response.is_valid;
  } else {
    ROS_ERROR("Failed to call service check_point");
    throw 0;
  }
}

bool create_sample(
  opt_control::Samples::Request  &req,
  opt_control::Samples::Response &res)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> generateX(X_MIN, X_MAX);
  std::uniform_real_distribution<> generateY(Y_MIN, Y_MAX);
  std::uniform_real_distribution<> generateAngle(- M_PI, M_PI);

  float x, y;
  std::vector<double> samples;
  for (int i = 0; i < req.nbSamples; i++){
    // Create an (x, y) position while checking for obstacles
    try {
      do {
        x = generateX(gen);
        y = generateY(gen);
      } while (!checkValidity(x, y));
    } catch(int e) {
      return false;
    }

    samples.push_back(x);
    samples.push_back(y);
    samples.push_back(generateAngle(gen));
  }

  res.samples = samples;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sampling_service");
  ros::NodeHandle n;

  if (CHECK_OBSTACLES){
    ros::service::waitForService("check_point", 5000);
  }

  ros::ServiceServer service = n.advertiseService(
    "create_samples", create_sample);
  ROS_INFO("Ready to create samples.");
  ros::spin();

  return 0;
}

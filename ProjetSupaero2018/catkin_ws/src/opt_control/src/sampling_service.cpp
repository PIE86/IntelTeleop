#include <random>
#include <math.h>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include "obstacles/CheckPoint.h"

#include "opt_control/Samples.h"

bool checkValidity(float x, float y){
  ros::NodeHandle n;
  ros::ServiceClient client = \
    n.serviceClient<obstacles::CheckPoint>("check_point");
  obstacles::CheckPoint srv;
  srv.request.x = x;
  srv.request.y = y;

  if (client.call(srv)) {
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
  std::uniform_real_distribution<> generateFloat(0.0, 20);
  std::uniform_real_distribution<> generateAngle(- M_PI, M_PI);

  float x, y;
  std::vector<geometry_msgs::Point> samples;
  for (int i = 0; i < req.nbSamples; i++){
    geometry_msgs::Point sample;

    // Create an (x, y) position while checking for obstacles
    try {
      do {
        x = generateFloat(gen);
        y = generateFloat(gen);
      } while (!checkValidity(x, y));
    } catch(int e) {
      return false;
    }

    sample.x = x;
    sample.y = y;
    sample.z = generateAngle(gen);

    samples.push_back(sample);
  }

  res.samples = samples;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sampling_service");
  ros::NodeHandle n;

  ros::service::waitForService("check_point", 5000);

  ros::ServiceServer service = n.advertiseService(
    "create_samples", create_sample);
  ROS_INFO("Ready to create samples.");
  ros::spin();

  return 0;
}

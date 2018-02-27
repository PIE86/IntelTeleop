#include <random>
#include <math.h>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include "opt_control/Samples.h"

bool create_sample(
  opt_control::Samples::Request  &req,
  opt_control::Samples::Response &res)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> generateFloat(0.0, 50.0);
  std::uniform_real_distribution<> generateAngle(- M_PI, M_PI);

  std::vector<geometry_msgs::Point> samples;
  for (int i = 0; i < req.nbSamples; i++){
    geometry_msgs::Point sample;
    sample.x = generateFloat(gen);
    sample.y = generateFloat(gen);
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

  ros::ServiceServer service = n.advertiseService(
    "create_samples", create_sample);
  ROS_INFO("Ready to create samples.");
  ros::spin();

  return 0;
}

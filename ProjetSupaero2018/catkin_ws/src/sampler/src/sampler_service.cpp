#include <random>
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include "sampler/Sample.h"

bool create_sample(
  sampler::Sample::Request  &req,
  sampler::Sample::Response &res)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> generateFloat(0.0, 50.0);
  std::uniform_real_distribution<> generateAngle(- M_PI, M_PI);

  geometry_msgs::Point sample;
  sample.x = generateFloat(gen);
  sample.y = generateFloat(gen);
  sample.z = generateAngle(gen);

  res.sample = sample;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sampler_service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService(
    "create_sample", create_sample);
  ROS_INFO("Ready to create samples.");
  ros::spin();

  return 0;
}

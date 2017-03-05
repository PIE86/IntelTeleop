#include <ros/ros.h>

#include "hector_uav_msgs/MotorPWM.h"
#include <stdint.h>
#include <inttypes.h>
#include <vector>
std::vector<unsigned char> pwmsHigh (4, 255); // MAX POWER
std::vector<unsigned char> pwmsLow (4, 80); // MIN POWER

int main(int argc, char **argv)
{
  ros::init(argc, argv, "user_interface");
  ros::NodeHandle n;

  ros::Rate loop_rate(50);

  ros::Publisher motor_commander_1 = n.advertise<hector_uav_msgs::MotorPWM>("/motor_pwm", 100);
//  pwmsHigh[ 2 ] = pwmsHigh[ 3 ] = 0;
  sleep( 5 );
  int i = 0;
  while (ros::ok()) // Check for ctrl+c and ROS
  {
    hector_uav_msgs::MotorPWM msg1;
    msg1.pwm = pwmsLow;
    motor_commander_1.publish(msg1); // Publish msg
    /*sleep(2);
    hector_uav_msgs::MotorPWM msg1Stop;
    motor_commander_1.publish(msg1Stop); // Publish msg
    sleep(2);*/
    ros::spinOnce();

    if( i++ > 250 )
    {
      pwmsLow[0] = i > 1500 ? 120 : 100;
      pwmsLow[1] = i > 1500 ? 120 : 100;
      pwmsLow[2] = i > 1500 ? 105 : 100;
      pwmsLow[3] = i > 1500 ? 105 : 100;
    }


    loop_rate.sleep();
  }

  return 0;
}
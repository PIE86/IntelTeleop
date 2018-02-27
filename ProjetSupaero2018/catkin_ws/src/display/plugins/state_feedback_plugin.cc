#ifndef _STATE_FEEDBACK_CONTROL_PLUGIN_HH_
#define _STATE_FEEDBACK_CONTROL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <gazebo/math/Pose.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <utils/State.h>

#define PI 3.14159265

namespace gazebo

{
  class StateFeedbackPlugin : public ModelPlugin
  {

  	public: StateFeedbackPlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

  		this->model = _model;
      int argc;
      char **argv;
      ros::init(argc, argv, "talker");
      ros::NodeHandle n;
      ros::Publisher state_pub = n.advertise<utils::State>("state", 1000);
      ros::Rate loop_rate(10);
      while (ros::ok())
      {
        utils::State state;
        gazebo::math::Pose pose;
        geometry_msgs::Vector3 pos_msg;
        geometry_msgs::Quaternion rot_msg;

        pose = this->model->GetWorldPose();
        pos_msg.x = pose.pos.x;
        pos_msg.y = pose.pos.y;
        pos_msg.z = pose.pos.z;
        rot_msg.x = pose.rot.x;
        rot_msg.y = pose.rot.y;
        rot_msg.z = pose.rot.z;
        rot_msg.w = pose.rot.w;

        state.pos = pos_msg;
        state.rot = rot_msg;

        state_pub.publish(state);
        ros::spinOnce();
        loop_rate.sleep();
      }
    }

    private: physics::ModelPtr model;

  };

  GZ_REGISTER_MODEL_PLUGIN(StateFeedbackPlugin)
}

#endif

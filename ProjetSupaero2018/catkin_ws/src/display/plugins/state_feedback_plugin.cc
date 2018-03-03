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
#include <ros/callback_queue.h>
#include <thread>
#include "std_msgs/Int32MultiArray.h"

namespace gazebo
{
  class StateFeedbackPlugin : public ModelPlugin
  {
  	// Empty constructor
  	public: StateFeedbackPlugin() {}


		// On launch (= once)
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
			// Model shortcut
  		this->model = _model;

			// Create node associated to plugin & subscribe
      this->rosNode.reset(new ros::NodeHandle("car_control"));
      this->state_pub = this->rosNode->advertise<std_msgs::Int32MultiArray>("state", 100);
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      	std::bind(&StateFeedbackPlugin::OnUpdate, this));
      this->rosQueueThread = std::thread(std::bind(&StateFeedbackPlugin::QueueThread, this));
    }


    public: void OnUpdate()
    {
    	// Publish rate
      ros::Rate loop_rate(10);

      // Data to be sent
      std_msgs::Int32MultiArray state;
      state.data.clear();
      
      math::Pose pose = this->model->GetWorldPose();

      state.data.push_back(pose.pos.x);
      state.data.push_back(pose.pos.y);
      state.data.push_back(pose.rot.GetAsEuler().z);

			// Publish data
      state_pub.publish(state);
    }

		// ROS helper function that processes messages (compulsory)
    private: void QueueThread()
  	{
  	  static const double timeout = 0.01;
  	  while (this->rosNode->ok())
  	  {
  	    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  	  }
  	}

    private: physics::ModelPtr model;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Publisher state_pub;
    private: ros::CallbackQueue rosQueue;
  	private: std::thread rosQueueThread;
    private: event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(StateFeedbackPlugin)
}

#endif

#ifndef _CAR_CONTROL_PLUGIN_HH_
#define _CAR_CONTROL_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Quaternion.hh>

#include <stdio.h>

#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <display/Command.h>

#include <math.h>

#define VERBOSE false

namespace gazebo
{
	class CarControlPlugin : public ModelPlugin
	{
		// Empty constructor
		public: CarControlPlugin() {}

		// On launch (= once)
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			// Model shortcut
			this->model = _model;

			// Set velocity according to sdf file (if relevant)
			this->velocity = math::Vector3(_sdf->Get<double>("X_velocity"),
										   _sdf->Get<double>("Y_velocity"),
										   0);
			this->SetVelocity(this->velocity);

			// Set orientation according to sdf file (if relevant) -- in degrees
			// Warning: this is an angle, not an angular rate
			// double thetaInit = _sdf->Get<double>("theta");
			// this->SetOrientation(thetaInit);

			// Create node associated to plugin
			this->rosNode.reset(new ros::NodeHandle("car_control"));
			// Subscribe to node "name" with queue size as 2nd param
			this->rosSub = this->rosNode->subscribe("command", 100, &CarControlPlugin::OnRosMsg, this);
			this->rosQueueThread = std::thread(std::bind(&CarControlPlugin::QueueThread, this));
		}

		public: void SetOrientation(const double& thetaRad)
		{
			//double thetaRad = _thetaDegree * M_PI/180;
			math::Pose initPose(this->model->GetWorldPose().pos,
													math::Quaternion(0, 0, thetaRad));
			this->model->SetWorldPose(initPose);
		}


		public: void SetVelocity(const gazebo::math::Vector3 & vel)
		{
			// Set linear velocity as vector
			this->model->SetLinearVel(vel);
			if (VERBOSE){
				gzmsg << "Linear velocity set to: " << vel << "\n";
			}
		}


		public: void SetAngularVelocity(const double & omega)
		{
			// Set angular velocity omega around axis z
			this->model->SetAngularVel({0, 0, omega});
			if (VERBOSE){
				gzmsg << "Angular velocity set to: " << omega << "\n";
			}
		}


		// Is triggered on every ROS message received (i.e. command)
		public: void OnRosMsg(const display::CommandConstPtr &_msg)
		{
			// Get message
			double v = _msg->u[0];
			double omega = _msg->u[1];

			// Set angular velocity
			this->SetAngularVelocity(omega);

			// Set velocity
			// Get yaw from pose
			math::Quaternion pose = this->model->GetWorldPose().rot;
			double yaw = pose.GetYaw();

			// Set velocity as (vx, vy)
			// this->velocity.Set(-v*sin(yaw), v*cos(yaw), 0);
			this->velocity.Set(v*cos(yaw), v*sin(yaw), 0);
			this->SetVelocity(this->velocity);
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

			private: gazebo::math::Vector3 velocity;
			private: gazebo::math::Vector3 orientation;
			private: physics::ModelPtr model;
			private: std::unique_ptr<ros::NodeHandle> rosNode;
			private: ros::Subscriber rosSub;
			private: ros::CallbackQueue rosQueue;
			private: std::thread rosQueueThread;
	};

	GZ_REGISTER_MODEL_PLUGIN(CarControlPlugin)
}

#endif

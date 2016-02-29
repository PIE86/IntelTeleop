#include "viewer.h"

#include <iostream>
#include <unistd.h>
#include <omniORB4/CORBA.h>
#include <string>
#include <cmath>
#include "environmentparser.h"

using namespace std;
typedef CORBA::ULong WindowID;
using namespace Eigen;

Viewer::Viewer(): client()
{
    WindowID w_id=client.createWindow("window");
	client.createSceneWithFloor("/world");
	client.addSceneToWindow("/world",w_id);

	se3Drone=se3::SE3::Identity();
}


void Viewer::createEnvironment(std::vector<Ecylinder> cylinder_list)
{ 
	float yellow[4] = {1,1,0.1,1.};
	se3::SE3 se3position=se3::SE3::Identity();
	int i=1;
	for(Ecylinder cyl : cylinder_list) 
	{
		string n="/world/cylinder"+std::to_string(i);
		const char* name=n.c_str();
		client.addCylinder(name, cyl.radius, sqrt(pow(cyl.x2-cyl.x1,2)+pow(cyl.y2-cyl.y1,2)+pow(cyl.z2-cyl.z1,2)),  yellow);


		se3position.translation({(cyl.x1+cyl.x2)/2,(cyl.y1+cyl.y2)/2,(cyl.z1+cyl.z2)/2});

		float x=cyl.x2-cyl.x1;
		float y=cyl.y2-cyl.y1;
		float z=cyl.z2-cyl.z1;

		// Rotation en z
		float theta=atan2(y,x);
		Matrix3f m_z(3,3);
		m_z(0,0)=cos(theta);
		m_z(0,1)=-sin(theta);
		m_z(0,2)=0;

		m_z(1,0)=sin(theta);
		m_z(1,1)=cos(theta);
		m_z(1,2)=0.0;

		m_z(2,0)=0;
		m_z(2,1)=0;
		m_z(2,2)=1;

		
		// Rotation en y
		float phi=atan2(x,z);
		Matrix3f m_y(3,3);
		m_y(0,0)=cos(phi);
		m_y(0,1)=0;
		m_y(0,2)=-sin(phi);

		m_y(1,0)=0;
		m_y(1,1)=1;
		m_y(1,2)=0;

		m_y(2,0)=sin(phi);
		m_y(2,1)=0;
		m_y(2,2)=cos(phi);

		
		// Rotation en x
		float psi=atan2(z,sqrt(pow(x,2)+pow(y,2)));
		Matrix3f m_x(3,3);
		m_x(0,0)=1.0;
		m_x(0,1)=0;
		m_x(0,2)=0.0;

		m_x(1,0)=0;
		m_x(1,1)=cos(psi);
		m_x(1,2)=-sin(psi);

		m_x(2,0)=0.0;
		m_x(2,1)=sin(psi);
		m_x(2,2)=cos(psi);


		se3position.rotation(m_z*m_x);
		client.applyConfiguration(name, se3position) ;
		i=i+1;
    }
	client.refresh();
}

void Viewer::createDrone(const char*  t)
{ 

	bool a=client.addMesh("/world/drone", t) ;
	if(a==0)
	{
		std::cout << "Erreur de chargement du modèle du drone"<< std::endl;
	}

	se3::SE3 se3position=se3::SE3::Identity();
	se3position.translation({0.0,0.0,1.0});
	client.applyConfiguration("/world/drone", se3position) ;
	client.refresh();
}

void Viewer::moveDrone(float x, float y, float z, float roll, float pitch, float yaw)
{ 
	se3Drone.translation({x,y,z});

	// Roll
	Matrix3f m_roll(3,3);
	m_roll(0,0)=1;
	m_roll(0,1)=0;
	m_roll(0,2)=0;

	m_roll(1,0)=0;
	m_roll(1,1)=cos(roll);
	m_roll(1,2)=-sin(roll);

	m_roll(2,0)=0;
	m_roll(2,1)=sin(roll);
	m_roll(2,2)=cos(roll);


	// Pitch
	Matrix3f m_pitch(3,3);
	m_pitch(0,0)=cos(pitch);
	m_pitch(0,1)=0;
	m_pitch(0,2)=sin(pitch);

	m_pitch(1,0)=0;
	m_pitch(1,1)=1;
	m_pitch(1,2)=0;

	m_pitch(2,0)=-sin(pitch);
	m_pitch(2,1)=0;
	m_pitch(2,2)=cos(pitch);


	// Yaw
	Matrix3f m_yaw(3,3);
	m_yaw(0,0)=cos(yaw);
	m_yaw(0,1)=-sin(yaw);
	m_yaw(0,2)=0;

	m_yaw(1,0)=sin(yaw);
	m_yaw(1,1)=cos(yaw);
	m_yaw(1,2)=0;

	m_yaw(2,0)=0;
	m_yaw(2,1)=0;
	m_yaw(2,2)=1;

	se3Drone.rotation()=se3Drone.rotation()*m_yaw*m_pitch*m_roll;
	client.applyConfiguration("/world/drone", se3Drone) ;
	client.refresh();
}


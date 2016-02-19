#include "viewer.h"

#include <iostream>
#include <unistd.h>
#include <omniORB4/CORBA.h>
#include <string>

using namespace std;
typedef CORBA::ULong WindowID;

Viewer::Viewer(): client()
{
    WindowID w_id=client.createWindow("window");
	client.createSceneWithFloor("/world");
	client.addSceneToWindow("/world",w_id);

	
}


void Viewer::createEnvironment()
{ 

	float yellow[4] = {1,1,0.1,1.};
	client.addCylinder("/world/cylinder", 1.0, 0.3, yellow);

	se3::SE3 se3position=se3::SE3::Identity();
	se3position.translation({1.0,1.0,3.0});
	client.applyConfiguration("/world/cylinder", se3position) ;
	client.refresh();
	usleep(50000);
}

void Viewer::createDrone(const char*  t)
{ 

	bool a=client.addMesh("/world/drone", t) ;
	if(a==0)
	{
		std::cout << "Erreur de chargement du modèle du drone"<< std::endl;
	}

	se3::SE3 se3position=se3::SE3::Identity();
	se3position.translation({1.0,1.0,1.0});
	client.applyConfiguration("/world/drone", se3position) ;
	client.refresh();
	usleep(50000);
}

void Viewer::moveDrone(float x, float y, float z)
{ 
	se3::SE3 se3position=se3::SE3::Identity();
	se3position.translation({x,y,z});
	client.applyConfiguration("/world/drone", se3position) ;
	client.refresh();
	usleep(50000);
}

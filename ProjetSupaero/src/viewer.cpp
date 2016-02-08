#include "viewer.h"

#include <iostream>
#include <unistd.h>
#include <omniORB4/CORBA.h>

typedef CORBA::ULong WindowID;

Viewer::Viewer(): client()
{
    WindowID w_id=client.createWindow("window");
	client.createSceneWithFloor("/world");
	client.addSceneToWindow("/world",w_id);

	
}

void Viewer::createDrone()
{ 

	float white[4] = {1.,1.,1.,1.};
	client.addCylinder("/world/drone", 1.0, 0.3, white);
	
	se3::SE3 se3position=se3::SE3::Identity();
	se3position.translation({1.0,1.0,1.0});
	client.applyConfiguration("/world/drone", se3position) ;
	client.refresh();
	usleep(50000);
}

void Viewer::moveDrone(float x, float y, float z)
{ 

	float white[4] = {1.,1.,1.,1.};
	client.addCylinder("/world/drone", 1.0, 0.3, white);
	
	se3::SE3 se3position=se3::SE3::Identity();
	se3position.translation({x,y,z});
	client.applyConfiguration("/world/drone", se3position) ;
	client.refresh();
	usleep(50000);
}

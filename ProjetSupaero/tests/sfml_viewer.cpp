#define AVOID_SINGULARITIES 2

#include "gepetto/viewer/corba/client.hh"
#include "viewer.h"
#include "input.h"

// Demonstration d'affichage de l'environnement a partir d'un fichier xml

int main( ){

	Viewer viewer;
	
	const char*  dronename=PIE_SOURCE_DIR"/data/quadrotor_base.stl";
    viewer.createDrone(dronename);
	
    Input input;

	float x=1.0;
	float y=0.0;
	float z=1.0;
	float roll=0.0;
	float pitch=0.0;
	float yaw=0.0;

	/*
	// Translation Test
	std::cout << "Translation Test" << std::endl;
	for(float i=0;i<200;i++)
	{
		x=0.0+i/200;
		y=0.0-i/120;
		z=1.0+i/140;
		
		viewer.moveDrone(x,y,z,roll,pitch,yaw);
		usleep(50000);
	}
	*/
	while(true)
	{
		std::array<double,3> list_inputs=input.getReference();
		//std::cout << list_inputs[0] << std::endl;
		x=x+list_inputs[0]/200;
		y=y+list_inputs[1]/200;
		z=z+list_inputs[2]/200;
		viewer.moveDrone(x,y,z,roll,pitch,yaw);

		usleep(50000);
	}
    return 0;
}


#define AVOID_SINGULARITIES 2

#include "gepetto/viewer/corba/client.hh"
#include "viewer.h"

// Demonstration d'affichage du drone dans le viewer et vérification des translations
// et rotations.

int main( ){

	Viewer viewer;

	const char*  dronename=PIE_SOURCE_DIR"/data/quadrotor_base.stl";
    viewer.createDrone(dronename);
	

	float x=1.0;
	float y=0.0;
	float z=1.0;
	float roll=0.0;
	float pitch=0.0;
	float yaw=0.0;

	usleep(2000000);

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
	usleep(5000000);

	// Yaw Test
	std::cout << "Yaw Test" << std::endl;
	for(float i=0;i<100;i++)
	{
		yaw=0.02;
		
		viewer.moveDrone(x,y,z,roll,pitch,yaw);
		usleep(50000);
	}
	yaw=0.0;
	usleep(3000000);


	// Pitch Test
	std::cout << "Pitch Test" << std::endl;
	for(float i=0;i<100;i++)
	{
		pitch=0.02;
		
		viewer.moveDrone(x,y,z,roll,pitch,yaw);
		usleep(50000);
	}
	pitch=0.0;
	usleep(3000000);


	// Roll Test
	std::cout << "Roll Test" << std::endl;
	for(float i=0;i<100;i++)
	{
		roll=0.02;
		
		viewer.moveDrone(x,y,z,roll,pitch,yaw);
		usleep(50000);
	}

    return 0;
}


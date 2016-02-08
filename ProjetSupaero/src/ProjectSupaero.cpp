#ifndef MAIN
#define MAIN

#include <gepetto/viewer/corba/client.hh>
#include "viewer.h"
#include <pthread.h>

int main( ){

	Viewer viewer;

    viewer.createDrone();

	for(float i=0;i<50;i++)
	{
		float x=1.0;
		float y=1.0;
		float z=1.0+i/20;
		viewer.moveDrone(x,y,z);
	}
    return 0;
}


#endif

#ifndef MAIN
#define MAIN

#include <gepetto/viewer/corba/client.hh>
#include "viewer.h"
#include <pthread.h>
#include "environmentparser.h"
#include <string>
#include <vector>

int main( ){

	Viewer viewer;
	std::string a="env.xml";
	EnvironmentParser e(a);
	/*
	std::vector<Ecylinder> cylinderList = e.readData();
	*/

    //viewer.createDrone();

	for(float i=0;i<50;i++)
	{
		float x=1.0;
		float y=1.0;
		float z=1.0+i/20;
		//viewer.moveDrone(x,y,z);
	}
    return 0;
}


#endif

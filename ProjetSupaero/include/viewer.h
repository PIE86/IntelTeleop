#ifndef VIEWER
#define VIEWER

#include "gepetto/viewer/corba/client.hh"
#include <string>
#include "environmentparser.h"

using namespace graphics;
using namespace corbaServer;
using namespace std;

typedef CORBA::ULong WindowID;

class Viewer 
{
	public:
		Viewer();
		void createEnvironment(std::vector<Ecylinder> cylinder_list);
		void createDrone(const char*  t);
		void moveDrone(float x, float y, float z, float roll, float pitch, float yaw);

	private:
		ClientCpp client;
		WindowID w_id;
		se3::SE3 se3Drone;
};

#endif


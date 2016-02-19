#ifndef VIEWER
#define VIEWER

#include "gepetto/viewer/corba/client.hh"
#include <string>

using namespace graphics;
using namespace corbaServer;
using namespace std;

typedef CORBA::ULong WindowID;

class Viewer 
{
	public:
		Viewer();
		void createEnvironment();
		void createDrone(const char*  t);
		void moveDrone(float x, float y, float z);

	private:
		ClientCpp client;
		WindowID w_id;
};

#endif


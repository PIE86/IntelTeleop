#ifndef VIEWER
#define VIEWER

#include "gepetto/viewer/corba/client.hh"

using namespace graphics;
using namespace corbaServer;

typedef CORBA::ULong WindowID;

class Viewer 
{
	public:
		Viewer();
		void createDrone();
		void moveDrone(float x, float y, float z);

	private:
		ClientCpp client;
		WindowID w_id;
};

#endif


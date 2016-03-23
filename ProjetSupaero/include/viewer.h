#ifndef VIEWER
#define VIEWER

#include "gepetto/viewer/corba/client.hh"
#include <string>
#include "environmentparser.h"

using namespace graphics;
using namespace corbaServer;
using namespace std;

typedef CORBA::ULong WindowID;


enum class Axis
{
	X,Y,Z
};

/**
 * @brief The Viewer class is an interface to the Gepetto server. It provides methods to initialise the client, drone
 * and cylinders. It also provides a method to move the drone and display an arrow next to the drone.
 */
class Viewer
{
public:
	/**
	 * @brief Viewer Initialises the gepetoo client and creates the world scene
	 */
	Viewer();

	/**
	 * @brief createEnvironment Create gepetto cylinders and draw them for each set of coordinates in cylinder_list
	 * @param cylinder_list List of cylinders to create
	 */
	void createEnvironment(const std::vector<Ecylinder> &cylinder_list);

	/**
	 * @brief createDrone Create and initialise drone in gepetto
	 * @param filename Mesh file to load for the drone
	 */
	void createDrone(const char* filename);

	/**
	 * @brief moveDrone Set the drone's new position in space, using cartesian coordinates and roll-pitch-yaw angles
	 * @param x
	 * @param y
	 * @param z
	 * @param roll
	 * @param pitch
	 */
	void moveDrone(double x, double y, double z, double roll, double pitch, double yaw);

	/**
	 * @brief setArrow Sets the arrow's direction according the the speed commands in cartesian coordinates
	 * @param vx
	 * @param vy
	 * @param vz
	 */
	void setArrow(int vx, int vy, int vz);

private:
	ClientCpp client;
	WindowID w_id;
	se3::SE3 se3Drone;
	std::vector<Ecylinder> cylinders;

	/**
	 * @brief rotationMat Builds a rotation matrix from an angle and an axis
	 * @param angle Angle in radian
	 * @param axis Axis of rotation. Can be either of Axis::X, Axis::Y, Axis::Z
	 * @return the rotation matrix
	 */
	Eigen::Matrix3d rotationMat(double angle, Axis axis);
};

#endif


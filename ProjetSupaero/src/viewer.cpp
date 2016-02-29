#include "viewer.h"

#include <iostream>
#include <unistd.h>
#include <omniORB4/CORBA.h>
#include <string>
#include <cmath>

using namespace std;
typedef CORBA::ULong WindowID;
using namespace Eigen;

Viewer::Viewer(): client()
{
    WindowID w_id = client.createWindow("window");
    client.createSceneWithFloor("/world");
    client.addSceneToWindow("/world",w_id);

    se3Drone = se3::SE3::Identity();
    se3Drone.translation({0.0,0.0,1.0});
}


void Viewer::createEnvironment()
{

    float yellow[4] = {1.,1.,0.,1.};
    client.addCylinder("/world/cylinder", 1.f, 0.3f, yellow);

    se3::SE3 se3position = se3::SE3::Identity();
    se3position.translation({1.,1.,3.});
    client.applyConfiguration("/world/cylinder", se3position);
    client.refresh();
}

void Viewer::createDrone(const char*  t)
{
    if (!client.addMesh("/world/drone", t))
        std::cout << "Erreur de chargement du modèle du drone" << std::endl;

    se3::SE3 se3position = se3::SE3::Identity();
    se3position.translation({4.0,1.0,1.0});
    client.applyConfiguration("/world/drone", se3position) ;
    client.refresh();
}

void Viewer::moveDrone(float x, float y, float z, float roll, float pitch, float yaw)
{
    se3Drone.translation({x,y,z});

    // Roll
    Matrix3f m_roll(3,3);
    m_roll(0,0) = 1;
    m_roll(0,1) = 0;
    m_roll(0,2) = 0;

    m_roll(1,0) = 0;
    m_roll(1,1) = cos(roll);
    m_roll(1,2) = -sin(roll);

    m_roll(2,0) = 0;
    m_roll(2,1) = sin(roll);
    m_roll(2,2) = cos(roll);

    // Pitch
    Matrix3f m_pitch(3,3);
    m_pitch(0,0) = cos(pitch);
    m_pitch(0,1) = 0;
    m_pitch(0,2) = sin(pitch);

    m_pitch(1,0) = 0;
    m_pitch(1,1) = 1;
    m_pitch(1,2) = 0;

    m_pitch(2,0) = -sin(pitch);
    m_pitch(2,1) = 0;
    m_pitch(2,2) = cos(pitch);

    // Yaw
    Matrix3f m_yaw(3,3);
    m_yaw(0,0) = cos(yaw);
    m_yaw(0,1) = -sin(yaw);
    m_yaw(0,2) = 0;

    m_yaw(1,0) = sin(yaw);
    m_yaw(1,1) = cos(yaw);
    m_yaw(1,2) = 0;

    m_yaw(2,0) = 0;
    m_yaw(2,1) = 0;
    m_yaw(2,2) = 1;

    //cout << m_yaw << endl;
    se3Drone.rotation() = m_yaw * m_pitch * m_roll;
    client.applyConfiguration("/world/drone", se3Drone) ;
    client.refresh();
}


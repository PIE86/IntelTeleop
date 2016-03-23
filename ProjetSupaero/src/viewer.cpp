/**************************************************************************
Copyright 2016 Yoan BAILLAU, Thibault BARBIÉ, Zhengxuan JIA,
   Francisco PEDROSA-REIS, William RAKOTOMANGA, Baudouin ROULLIER

This file is part of ProjectSupaero.

ProjectSupaero is free software: you can redistribute it and/or modify
it under the terms of the lesser GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

ProjectSupaero is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
lesser GNU General Public License for more details.

You should have received a copy of the lesser GNU General Public License
along with ProjectSupaero.  If not, see <http://www.gnu.org/licenses/>.

**************************************************************************/


#include "viewer.h"
#include <iostream>
#include <unistd.h>
#include <omniORB4/CORBA.h>
#include <string>
#include <cmath>
#include "environmentparser.h"

typedef CORBA::ULong WindowID;
using namespace Eigen;

Viewer::Viewer(): client()
{
    // create a clent window and a world scene in it
    WindowID w_id = client.createWindow("window");
    client.createScene("/world");
    client.addSceneToWindow("/world",w_id);

    // initialise drone position
    se3Drone = se3::SE3::Identity();
    se3Drone.translation({0.,0.,2.});
}


void Viewer::createEnvironment(const std::vector<Ecylinder> &cylinder_list)
{
    // copy cylinders to memory
    cylinders = cylinder_list;

    // initialise color and position
    float yellow[4] = {1.f,1.f,.1f,1.f};
    se3::SE3 se3position = se3::SE3::Identity();
    int i = 1;

    // for each cylinder in the list, compute translation vector and rotation matrix, and create gepetto objects.
    for(Ecylinder cyl : cylinders)
    {
        string n = "/world/cylinder"+std::to_string(i++);
        const char* name = n.c_str();

        float dx = cyl.x2-cyl.x1;
        float dy = cyl.y2-cyl.y1;
        float dz = cyl.z2-cyl.z1;

        client.addCylinder(name, cyl.radius, sqrt(pow(dx,2.f)+pow(dy,2.f)+pow(dz,2.f)), yellow);

        se3position.translation({(cyl.x1+cyl.x2)/2.f,(cyl.y1+cyl.y2)/2.f,(cyl.z1+cyl.z2)/2.f});

        double theta = atan2(dy,dx);
        double phi = -atan2(sqrt(pow(dx,2)+pow(dy,2)),dz);
        Matrix3d m_z = rotationMat(theta, Axis::Z);
        Matrix3d m_y = rotationMat(phi, Axis::Y);

        se3position.rotation(m_z.cast<float>()*m_y.cast<float>());
        client.applyConfiguration(name, se3position) ;
    }
    client.refresh();
}

void Viewer::createDrone(const char*  filename)
{
    // load drone mesh
    bool a = client.addMesh("/world/drone", filename) ;
    if(a == 0)
        std::cout << "Erreur de chargement du modèle du drone"<< std::endl;

    // create gepetto object for the drone
    se3::SE3 se3position = se3::SE3::Identity();
    se3position.translation({0.,0.,1.});
    client.applyConfiguration("/world/drone", se3position);

    // create cylinder for the arrow
    float red[4] = {1.f,0.f,.0f,1.f};
    client.addCylinder("/world/arrow", .1f, 4.f, red);

    client.refresh();
}

void Viewer::moveDrone(double x, double y, double z, double roll, double pitch, double yaw)
{
    // This member function does not move the drone but the world around it. Indeed, we want the camera to be centered on the drone

    // first translate the cylinders
    se3::SE3 se3position = se3::SE3::Identity();

    // for each cylinder in the list, compute translation vector and rotation matrix, and move them
    for(unsigned int i=0 ; i<cylinders.size(); i++)
    {
        Ecylinder cyl = cylinders[i];
        string n = "/world/cylinder"+std::to_string(i+1);

        se3position.translation({(cyl.x1+cyl.x2)/2.f - (float)x,(cyl.y1+cyl.y2)/2.f - (float)y,(cyl.z1+cyl.z2)/2.f - (float)z});

        float dx = cyl.x2-cyl.x1;
        float dy = cyl.y2-cyl.y1;
        float dz = cyl.z2-cyl.z1;

        double theta = atan2(dy,dx);
        double phi = -atan2(sqrt(pow(dx,2)+pow(dy,2)),dz);
        Matrix3d m_z = rotationMat(theta, Axis::Z);
        Matrix3d m_y = rotationMat(phi, Axis::Y);

        se3position.rotation(m_z.cast<float>()*m_y.cast<float>());

        client.applyConfiguration(n.c_str(), se3position);
    }

    // compute rotation matrices for the drone
    Matrix3d m_roll = rotationMat(roll, Axis::X);
    Matrix3d m_pitch = rotationMat(-pitch, Axis::Y);
    Matrix3d m_yaw = rotationMat(yaw, Axis::Z);

    // apply rotation matrices to the drone
    se3Drone.rotation() = m_yaw.cast<float>() * m_pitch.cast<float>() * m_roll.cast<float>();
    client.applyConfiguration("/world/drone", se3Drone);
    client.refresh();
}

void Viewer::setArrow(int vx, int vy, int vz)
{
    auto dronePos = se3Drone.translation();
    se3::SE3 se3position = se3::SE3::Identity();

    // if there is no speed command, move the arrow far away
    if (vx == 0 && vy == 0 && vz == 0)
        se3position.translation({ 0.f,0.f,10000.f });
    else
    {
        // translate the arrow next to the drone
        se3position.translation({ dronePos[0] + 2.5f*(float)vx , dronePos[1] + 2.5f*(float)vy, dronePos[2] + 2.5f*(float)vz });

        // compute the rotation matrices
        double theta = atan2(vy,vx);
        double phi = -atan2(sqrt(pow(vx,2)+pow(vy,2)),vz);
        Matrix3d m_z = rotationMat(theta, Axis::Z);
        Matrix3d m_y = rotationMat(phi, Axis::Y);

        se3position.rotation(m_z.cast<float>()*m_y.cast<float>());
    }
    // apply translation and rotations
    client.applyConfiguration("/world/arrow", se3position);
    client.refresh();
}

Matrix3d Viewer::rotationMat(double angle, Axis axis)
{
    Matrix3d mat(3,3);
    double c = cos(angle), s = sin(angle);

    switch (axis)
    {
    case Axis::X:
        mat(0,0) = 1.;
        mat(0,1) = 0.;
        mat(0,2) = 0.;

        mat(1,0) = 0.;
        mat(1,1) = c;
        mat(1,2) = -s;

        mat(2,0) = 0.;
        mat(2,1) = s;
        mat(2,2) = c;
        break;

    case Axis::Y:
        mat(0,0) = c;
        mat(0,1) = 0.;
        mat(0,2) = -s;

        mat(1,0) = 0.;
        mat(1,1) = 1.;
        mat(1,2) = 0.;

        mat(2,0) = s;
        mat(2,1) = 0.;
        mat(2,2) = c;
        break;

    case Axis::Z:
        mat(0,0) = c;
        mat(0,1) = -s;
        mat(0,2) = 0.;

        mat(1,0) = s;
        mat(1,1) = c;
        mat(1,2) = 0.;

        mat(2,0) = 0.;
        mat(2,1) = 0.;
        mat(2,2) = 1.;
        break;

    default:
        break;
    }

    return mat;
}


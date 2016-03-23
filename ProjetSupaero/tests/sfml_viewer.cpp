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


#define AVOID_SINGULARITIES 2

#include "gepetto/viewer/corba/client.hh"
#include "viewer.h"
#include "input.h"

// Demonstration d'affichage de l'environnement a partir d'un fichier xml

int main( ){

    Viewer viewer;

    const char* dronename = PIE_SOURCE_DIR"/data/quadrotor_base.stl";
    viewer.createDrone(dronename);

    Input input;

    double x = 1.0;
    double y = 0.0;
    double z = 1.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    /*
    // Translation Test
    std::cout << "Translation Test" << std::endl;
    for(float i = 0;i<200;i++)
    {
        x = 0.0+i/200;
        y = 0.0-i/120;
        z = 1.0+i/140;

        viewer.moveDrone(x,y,z,roll,pitch,yaw);
        usleep(50000);
    }
    */
    while(true)
    {
        std::array<double,6> list_inputs = input.getReference();
        //std::cout << list_inputs[0] << std::endl;
        x = x + list_inputs[0]/200.;
        y = y + list_inputs[1]/200.;
        z = z + list_inputs[2]/200.;
        roll = list_inputs[3]/200.;
        pitch = list_inputs[4]/200.;
        yaw = list_inputs[5]/200.;
        viewer.moveDrone(x,y,z,roll,pitch,yaw);

        usleep(50000);
    }
    return 0;
}


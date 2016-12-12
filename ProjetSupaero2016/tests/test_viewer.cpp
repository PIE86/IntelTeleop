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
    Input input;

    const char* dronename = PIE_SOURCE_DIR"/data/quadrotor_base.stl";
    viewer.createDrone(dronename);

    double x = 1.0;
    double y = 0.0;
    double z = 1.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    while(true)
    {
        std::array<double,6> list_inputs = input.getReference();
        x += list_inputs[0]/10.;
        y += list_inputs[1]/10.;
        z += list_inputs[2]/10.;
        roll += list_inputs[3]/10.;
        pitch += list_inputs[4]/10.;
        yaw += list_inputs[5]/10.;
        viewer.moveDrone(x,y,z,roll,pitch,yaw);
        viewer.setArrow((list_inputs[0]>0) - (list_inputs[0]<0), (list_inputs[1]>0) - (list_inputs[1]<0), (list_inputs[2]>0) - (list_inputs[2]<0));

        usleep(50000);
    }
    return 0;
}


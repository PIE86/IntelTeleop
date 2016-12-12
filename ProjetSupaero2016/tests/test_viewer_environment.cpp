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
#include "environmentparser.h"

// Demonstration d'affichage de l'environnement a partir d'un fichier xml

int main( ){

    Viewer viewer;

//    const char*  dronename=PIE_SOURCE_DIR"/data/quadrotor_base.stl";
    std::string a=PIE_SOURCE_DIR"/data/envsavetest.xml";

    // Creation de l'environnement
    EnvironmentParser e;
    Epoint center1;
    Epoint center2;

    center1.x=0.f;
    center1.y=0.f;
    center1.z=0.f;
    center2.x=15.f;
    center2.y=0.f;
    center2.z=15.f;

    e.addCylinder(center1,center2,1.2f);
    e.save(a);


    // Chargement de l'environnement
    EnvironmentParser g(a);
    std::vector<Ecylinder> cylinder_list=g.readData();

    viewer.createEnvironment(cylinder_list);

    return 0;
}


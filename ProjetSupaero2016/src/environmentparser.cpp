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

#include "environmentparser.h"
#include <iostream>

// Macro to check XMLError validity
#ifndef XMLCheckResult
#define XMLCheckResult(a_eResult) if (a_eResult != tinyxml2::XML_SUCCESS) { std::cout << "Error: " <<  a_eResult << std::endl; }
#endif


EnvironmentParser::EnvironmentParser(): nbElements(0)
{
    // Create a root
    root = xmlDoc.NewElement("root");
    xmlDoc.InsertFirstChild(root);
}


EnvironmentParser::EnvironmentParser(const std::string &name): nbElements(0)
{
    // Load the file
    bool eResult = xmlDoc.LoadFile(name.c_str());
    XMLCheckResult(eResult);
    root = xmlDoc.FirstChildElement();
}

void EnvironmentParser::addCylinder(Epoint center1, Epoint center2, float radius)
{
    // Create a new element "cylinder"
    tinyxml2::XMLElement *cylinder = xmlDoc.NewElement("cylinder");
    root->InsertEndChild(cylinder);

    // Save its characteristics
    cylinder->SetAttribute("radius", radius);

    tinyxml2::XMLElement *point1 = xmlDoc.NewElement("center1");
    point1->SetAttribute("x", center1.x);
    point1->SetAttribute("y", center1.y);
    point1->SetAttribute("z", center1.z);
    cylinder->InsertEndChild(point1);

    tinyxml2::XMLElement *point2 = xmlDoc.NewElement("center2");
    point2->SetAttribute("x", center2.x);
    point2->SetAttribute("y", center2.y);
    point2->SetAttribute("z", center2.z);
    cylinder->InsertEndChild(point2);

    nbElements++;
}

void EnvironmentParser::save(std::string name)
{
    //  Specify the number of elements
    root->SetAttribute("nbElements", nbElements);

    bool eResult = xmlDoc.SaveFile(name.c_str());
    XMLCheckResult(eResult);
}

std::vector<Ecylinder> EnvironmentParser::readData()
{
    int i(0);
    tinyxml2::XMLElement *element;
    std::vector<Ecylinder> cylinderList;
    Ecylinder cylinder;
    tinyxml2::XMLElement *pCylinder = root->FirstChildElement();

    root->QueryIntAttribute("nbElements", &nbElements);

    for(i=0; i < nbElements; i++)
    {
        pCylinder->QueryFloatAttribute("radius", &(cylinder.radius));

        element = pCylinder->FirstChildElement("center1");
        element->QueryFloatAttribute("x", &(cylinder.x1));
        element->QueryFloatAttribute("y", &(cylinder.y1));
        element->QueryFloatAttribute("z", &(cylinder.z1));

        element = pCylinder->FirstChildElement("center2");
        element->QueryFloatAttribute("x", &(cylinder.x2));
        element->QueryFloatAttribute("y", &(cylinder.y2));
        element->QueryFloatAttribute("z", &(cylinder.z2));

        cylinderList.push_back(cylinder);

        pCylinder = pCylinder->NextSiblingElement();
    }

    return cylinderList;
}

int EnvironmentParser::getNbElements()
{
    return nbElements;
}

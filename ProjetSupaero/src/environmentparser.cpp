/* *******************************************************************************************************************************
Author: Yoann Baillau
Date: 27/01/2016
Version: 1.0
Last update: 27/01/2016
******************************************************************************************************************************* */

#include "environmentparser.h"

// Macro to check XMLError validity
#ifndef XMLCheckResult
#define XMLCheckResult(a_eResult) if (a_eResult != tinyxml2::XML_SUCCESS) { printf("Error: %i\n", a_eResult); }
#endif

EnvironmentParser::EnvironmentParser(): nbElements(0){
    // Create a root
    root = xmlDoc.NewElement("root");
    xmlDoc.InsertFirstChild(root);
}

EnvironmentParser::EnvironmentParser(std::string &name):nbElements(0){
    // Load the file
    tinyxml2::XMLError eResult = xmlDoc.LoadFile(name.c_str());
    XMLCheckResult(eResult);
    root = xmlDoc.FirstChildElement();
}

void EnvironmentParser::addCylinder(Epoint center1, Epoint center2, float radius) {
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

    tinyxml2::XMLError eResult = xmlDoc.SaveFile(name.c_str());
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

/* *******************************************************************************************************************************
Author: Yoann Baillau
Date: 27/01/2016
Version: 1.0
Last update: 27/01/2016
Description: Used to write or read an xml environment file
******************************************************************************************************************************* */

#ifndef ENVIRONMENTPARSER_H
#define ENVIRONMENTPARSER_H

#include <string>
#include <vector>
#include <iostream>
#include <tinyxml2.h>

struct Ecylinder {
    float x1, x2, y1, y2, z1, z2, radius;
};

struct Epoint {
    float x, y, z;
};

class EnvironmentParser
{
public:
    EnvironmentParser();                         // Create an empty XMLDocument
    EnvironmentParser(std::string &name);        // Load the XMLDocument with the given name (ex: "SavedData.xml")

    void addCylinder(Epoint center1, Epoint center2, float radius); // Add a cylinder to the environment from center1 to center2 with the given radius
    void save(std::string name);                                    // Save the XML document
    std::vector<Ecylinder> readData();                              // Store environment data in a vector
    int getNbElements();                                            // Return the number of elements in the environment


private:
    int nbElements;                         // Number of elements
    tinyxml2::XMLDocument xmlDoc;           // The opened document
    tinyxml2::XMLElement *root;             // The root of the document
};

#endif // ENVIRONMENTPARSER_H

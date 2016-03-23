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
#include <tinyxml2.h>

/**
 * @brief The Ecylinder struct describes a cylinder as two circular bases which centers coordinates
 * are (x1,y1,z1) and (x2,y2,z2) respectively, as well as a radius.
 */
struct Ecylinder
{
	float x1, x2, y1, y2, z1, z2, radius;
};

/**
 * @brief The Epoint struct describes a point in cartesian spatial coordinates.
 */
struct Epoint
{
	float x, y, z;
};

/**
 * @brief The EnvironmentParser class is an interface to the tinyxml2 library. Use the constructor to
 * load the XML file, readData() to parse it, and save() only if you addCylinder().
 */
class EnvironmentParser
{
public:
	/**
	 * @brief EnvironmentParser Creates an empty set of cylinders, to be completed with addCylinder
	 */
	EnvironmentParser();

	/**
	 * @brief EnvironmentParser Loads an XML document and prepares for parsing
	 * @param name Filename of the XML doc to load
	 */
	EnvironmentParser(const std::string &name);

	/**
	 * @brief addCylinder Manually add a cylinder that is not in the XML file
	 * @param center1 Center position of the first base
	 * @param center2 Center position of the second base
	 * @param radius Radius of the cylinder
	 */
	void addCylinder(Epoint center1, Epoint center2, float radius);

	/**
	 * @brief save Save cylinders in memory to an XML document (useful if addCylinder was called)
	 * @param name Filename to save to
	 */
	void save(std::string name);

	/**
	 * @brief readData Parse the loaded XML file to a vector of Ecylinder
	 * @return std::vector of Ecylinder
	 */
	std::vector<Ecylinder> readData();

	/**
	 * @brief getNbElements Get the number of cylinders in XML file and memory (from addCylinder)
	 * @return Number of cylinders
	 */
	int getNbElements();


private:
	int nbElements;                         // Number of elements
	tinyxml2::XMLDocument xmlDoc;           // The opened document
	tinyxml2::XMLElement *root;             // The root of the document
};

#endif // ENVIRONMENTPARSER_H

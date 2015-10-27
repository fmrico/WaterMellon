/*
 * Objects.h
 *
 *  Created on: 11/10/2015
 *      Author: paco
 */

#ifndef OBJECTS_H_
#define OBJECTS_H_

#include "ros/ros.h"

#include <json/json.h>

#include <map>
#include <string>
#include <iostream>
#include <fstream>


namespace wm_objects {

class Objects {
public:
	Objects();
	~Objects();

	void readFile(std::string filename);

	std::string getFilename(const std::string& obj_id);
	std::list<std::string> getAvailableObjects();

	void printObjects();

private:

	std::map<std::string,  std::string> objects_;
};

}

#endif /* LOCATIONS_H_ */

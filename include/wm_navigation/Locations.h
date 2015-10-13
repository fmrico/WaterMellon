/*
 * Locations.h
 *
 *  Created on: 11/10/2015
 *      Author: paco
 */

#ifndef LOCATIONS_H_
#define LOCATIONS_H_

#include "ros/ros.h"

#include <json/json.h>

#include <map>
#include <string>
#include <iostream>
#include <fstream>

#include <geometry_msgs/Pose.h>


namespace wm_navigation {

class Locations {
public:
	Locations();
	~Locations();

	void readFile(std::string filename);

	geometry_msgs::Pose getPose(std::string loc_id);
	void printLocations();

private:

	std::map<std::string,  geometry_msgs::Pose> locations_;
};

}

#endif /* LOCATIONS_H_ */

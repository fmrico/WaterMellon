/*
 * Locations.cpp
 *
 *  Created on: 11/10/2015
 *      Author: paco
 */

#include "wm_navigation/Locations.h"
#include <fstream>
#include <streambuf>
#include <string>


namespace wm_navigation {

Locations::Locations()
:locations_()
{


}

Locations::~Locations() {

}




void
Locations::readFile(std::string filename)
{

	if(filename!="")
	{

		std::ifstream t(filename.c_str());
		std::stringstream buffer;
		buffer << t.rdbuf();

		Json::Value root;   // will contains the root value after parsing.
		Json::Reader reader;
		bool parsingSuccessful = reader.parse( buffer.str(), root );
		if ( !parsingSuccessful )
		{
			// report to the user the failure and their locations in the document.
			std::cout  << "Failed to parse configuration\n"
					<< reader.getFormattedErrorMessages();
			return;
		}

		Json::Value locations = root["locations"];

		if(locations.isArray())
		{
			for(int i=0; i< locations.size(); i++)
			{
				if(locations[i].isMember("name") &&
				   locations[i].isMember("position") &&
				   locations[i].isMember("orientation") &&
				   locations[i].isMember("description"))
				{
					Json::Value name = locations[i]["name"];
					Json::Value position = locations[i]["position"];
					Json::Value orientation = locations[i]["orientation"];
					Json::Value description = locations[i]["description"];

					geometry_msgs::Pose pose;

					if(position.size()==3)
					{
						pose.position.x = position[0].asDouble();
						pose.position.y = position[1].asDouble();
						pose.position.z = position[2].asDouble();
					}else
						ROS_ERROR("Error parsing [%s][%s element]: field position requires 3 values", filename.c_str(),  name.asCString());

					if(orientation.size()==4)
					{
						pose.orientation.x = orientation[0].asDouble();
						pose.orientation.y = orientation[1].asDouble();
						pose.orientation.z = orientation[2].asDouble();
						pose.orientation.w = orientation[3].asDouble();
					}else
						ROS_ERROR("Error parsing [%s][%s element]: field orientation requires 4 values", filename.c_str(), name.asCString());

					locations_[name.asString()] = pose;
				}else
				{
					if(!locations[i].isMember("name"))
							ROS_ERROR("Error parsing [%s][%d element]: Missing field \"name\"", filename.c_str(), i);
					if(!locations[i].isMember("position"))
							ROS_ERROR("Error parsing [%s][%d element]: Missing field \"position\"", filename.c_str(), i);
					if(!locations[i].isMember("orientation"))
							ROS_ERROR("Error parsing [%s][%d element]: Missing field \"orientation\"", filename.c_str(), i);
					if(!locations[i].isMember("description"))
							ROS_ERROR("Error parsing [%s][%d element]: Missing field \"description\"", filename.c_str(), i);

				}
			}
		}

	}
}

void
Locations::printLocations()
{
	std::map<std::string,  geometry_msgs::Pose>::iterator it;

	for(it=locations_.begin(); it!=locations_.end();++it)
		std::cout<<"["<<it->first<<"] ("
		<<it->second.position.x<<", "
		<<it->second.position.y<<", "
		<<it->second.position.z
		<<") ("
		<<it->second.orientation.x<<", "
		<<it->second.orientation.y<<", "
		<<it->second.orientation.z<<", "
		<<it->second.orientation.w<<")"<<std::endl;

}

geometry_msgs::Pose
Locations::getPose(std::string loc_id)
{
	std::map<std::string,  geometry_msgs::Pose>::iterator it = locations_.find(loc_id);

	if(it != locations_.end())
		return locations_[loc_id];
	else
	{
		ROS_ERROR("Location [%s] not found", loc_id.c_str());
		return (geometry_msgs::Pose());
	}
}

} /* namespace wm_navigation */

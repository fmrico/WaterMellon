/*
 * Objects.cpp
 *
 *  Created on: 11/10/2015
 *      Author: paco
 */

#include "wm_objects/Objects.h"
#include <fstream>
#include <streambuf>
#include <string>


namespace wm_objects {

Objects::Objects()
:objects_()
{


}

Objects::~Objects() {

}




void
Objects::readFile(std::string filename)
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
			// report to the user the failure and their objects in the document.
			std::cerr  << "Failed to parse configuration\n"
					<< reader.getFormattedErrorMessages();
			return;
		}

		Json::Value objects = root["objects"];

		if(objects.isArray())
		{
			for(int i=0; i< objects.size(); i++)
			{
				if(objects[i].isMember("name") &&
						objects[i].isMember("filename"))
				{
					Json::Value name = objects[i]["name"];
					Json::Value filename = objects[i]["filename"];

					objects_[name.asString()] = filename.asString();
				}
			}
		}
	}
}

void
Objects::printObjects()
{
	std::map<std::string,  std::string>::iterator it;

	for(it=objects_.begin(); it!=objects_.end();++it)
		std::cout<<"["<<it->first<<"] ("
		<<it->second<<")"<<std::endl;

}

std::string
Objects::getFilename(const std::string& obj_id)
{
	std::map<std::string,  std::string>::iterator it = objects_.find(obj_id);

	if(it != objects_.end())
		return it->second;
	else
	{
		ROS_ERROR("Object [%s] does not exist", objects_[obj_id].c_str());
		return "";
	}
}

std::list<std::string>
Objects::getAvailableObjects()
{
	std::list<std::string> object_list;
	std::map<std::string,  std::string>::iterator it;

	for(it=objects_.begin(); it!=objects_.end();++it)
		object_list.push_back(it->first);
	return object_list;

}

} /* namespace wm_navigation */

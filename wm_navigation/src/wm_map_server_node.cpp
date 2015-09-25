/*
 * mapper_node.cpp
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */


#include "ros/ros.h"
#include "wm_navigation/WmMapServer.h"

#define USAGE "\nUSAGE: wm_map_server <map.pcl>\n" \
		"  map.pcl: inital pointcloud 3D map file to read\n"

using namespace wm_map_server;

int main(int argc, char **argv)
{
	ros::init(argc, argv, std::string("wm_map_server"));
	std::string mapFilename("");

	if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
		ROS_ERROR("%s", USAGE);
		exit(-1);
	}

	WmMapServer server;
	ros::spinOnce();

	if (argc == 2){
		mapFilename = std::string(argv[1]);
		if (!server.openFile(mapFilename)){
			ROS_ERROR("Could not open file %s", mapFilename.c_str());
			exit(1);
		}
	}

	try{
		ros::spin();
	}catch(std::runtime_error& e){
		ROS_ERROR("wm_map_server exception: %s", e.what());
		return -1;
	}

	return 0;

}

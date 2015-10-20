/*
 * mapper_node.cpp
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */


#include "ros/ros.h"

#include <wm_objects/WmObjectTrainer.h>

using namespace wm_objects;

int main(int argc, char **argv)
{
	ros::init(argc, argv, std::string("wm_objects"));
	ros::NodeHandle n;

	ros::Rate loop_rate(5);

	WmObjectTrainer wm_object_trainer;

	try{
		while ( ros::ok())
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
	}catch(std::runtime_error& e){
		ROS_ERROR("wm_object_node exception: %s", e.what());
		return -1;
	}

	return 0;

}

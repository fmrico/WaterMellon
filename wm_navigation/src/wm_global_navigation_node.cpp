/*
 * mapper_node.cpp
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */


#include "ros/ros.h"

#include "wm_navigation/WmGlobalNavigation.h"

using namespace wm_navigation;

int main(int argc, char **argv)
{
	ros::init(argc, argv, std::string("wm_global_navigation"));
	ros::NodeHandle n;

  ros::Rate loop_rate(5);

  WmGlobalNavigation gnav;

  try{
	   while ( ros::ok())
	   {

		   gnav.step();

       ros::spinOnce();
       loop_rate.sleep();
     }
	}catch(std::runtime_error& e){
		ROS_ERROR("wm_map_server exception: %s", e.what());
		return -1;
	}

	return 0;

}

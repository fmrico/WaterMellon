
#include "ros/ros.h"

#include <wm_objects/WmObjectRecognition.h>

using namespace wm_object;

int main(int argc, char **argv)
{
	ros::init(argc, argv, std::string("wm_object_recognition"));
	ros::NodeHandle n;

	ros::Rate loop_rate(5);

	WmObjectRecognition wm_object_recognition;

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

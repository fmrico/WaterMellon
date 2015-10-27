
#include "ros/ros.h"

#include <wm_objects/WmObjectCapture.h>

#define USAGE "\nUSAGE: wm_object_node <object.pcl>\n" \
		"  map.pcl: inital pointcloud 3D map file to read\n"

using namespace wm_objects;

int main(int argc, char **argv)
{
	ros::init(argc, argv, std::string("wm_object_capture"));
	ros::NodeHandle n;

	std::string objectFilename("");

	if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
		ROS_ERROR("%s", USAGE);
		exit(-1);
	}

	ros::Rate loop_rate(5);

	WmObjectCapture wm_object_trainer;

	if (argc == 2){
		objectFilename = std::string(argv[1]);
		if (!wm_object_trainer.openFile(objectFilename)){
			ROS_ERROR("Could not open file %s", objectFilename.c_str());
			exit(1);
		}
	}

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


#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/impl/point_types.hpp"
#include "pcl/point_types_conversion.h"
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>

#include <watermellon/GetObject.h>
#include <pcl/io/ply_io.h>

#define USAGE "\nUSAGE: wm_object_saver <objectfile.pcd>\n"

using namespace std;


class ObjectSaver{
public:
	ObjectSaver(const std::string& objectname)
{
		ros::NodeHandle n;
		std::string servname = "object";

		ROS_INFO("Requesting the object from %s...", n.resolveName(servname).c_str());

		watermellon::GetObject::Request req;
		watermellon::GetObject::Response resp;

		while(n.ok() && !ros::service::call(servname, req, resp))
		{
			ROS_WARN("Request to %s failed; trying again...", n.resolveName(servname).c_str());
			usleep(1000000);
		}

		if (n.ok()){

			pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>); // input cloud for filtering and ground-detection

			pcl_conversions::toPCL(resp.object, *cloud);
			pcl::fromPCLPointCloud2 (*cloud, *pc);

			if (pcl::io::savePLYFile<pcl::PointXYZRGB> (objectname, *pc) != -1)
			{
				ROS_INFO("Object saved (%zu pts)", cloud->data.size());
			}else
			{
				ROS_ERROR ("Couldn't write file %s", objectname.c_str());
			}


		}

}

};

int main(int argc, char** argv){
	ros::init(argc, argv, "wm_object_saver");
	std::string objectFilename("");

	if (argc == 2)
		objectFilename = std::string(argv[1]);
	else{
		ROS_ERROR("%s", USAGE);
		exit(1);
	}

	try{
		ObjectSaver ms(objectFilename);
	}catch(std::runtime_error& e){
		ROS_ERROR("wm_object_saver exception: %s", e.what());
		exit(2);
	}

	exit(0);
}

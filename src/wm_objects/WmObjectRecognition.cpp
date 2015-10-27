/*
 * WmObjectRecognition.cpp
 *
 *  Created on: 26/10/2015
 *      Author: paco
 */

#include "wm_objects/WmObjectRecognition.h"

namespace wm_object {

WmObjectRecognition::WmObjectRecognition()
:
							pairWidth_(30.0),
							voxelSize_(4.0),
							recognition_(pairWidth_, voxelSize_),
							objects_filenames_()
{
	ros::NodeHandle private_nh("~");
	std::string dir;
	std::string loc_file;

	if(private_nh.hasParam("objects_dir"))
		private_nh.param("objects_dir", dir,dir);
	else
	{
		ROS_ERROR("Objects DIR location incorrect");
		return;
	}


	if(private_nh.hasParam("objects_file"))
		private_nh.param("objects_file", loc_file,loc_file);
	else
	{
		ROS_ERROR("Objects location incorrect");
		return;
	}

	loc_file = dir+loc_file;

	ROS_INFO("Loading objects from [%s]", loc_file.c_str());

	objects_filenames_.readFile(loc_file);
	objects_filenames_.printObjects();

	std::list<std::string> objects = objects_filenames_.getAvailableObjects();

	for(std::list<std::string>::iterator it=objects.begin(); it!=objects.end(); ++it)
	{
		std::string obj_filename = dir+objects_filenames_.getFilename(*it);

		ROS_INFO("Training object [%s] from [%s]", it->c_str(), obj_filename.c_str());
		loadModelFromFile(obj_filename, *it);
		ROS_INFO("Done");
	}

}

WmObjectRecognition::~WmObjectRecognition()
{

}

void
WmObjectRecognition::loadModelFromFile(const std::string& filename, const std::string& objectname)
{
	bool success;
	int status;
	double normalRadius_ = 15.0;
	double successProbability_ = 0.99;
	std::cerr<<"1";
	pcl::PCLPointCloud2 cloudReadFromFile;

	status = pcl::io::loadPLYFile(filename, cloudReadFromFile);
	pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloudPtr (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr objectNormalsPtr (new pcl::PointCloud<pcl::Normal>());
	pcl::fromPCLPointCloud2(cloudReadFromFile, *objectCloudPtr);
	normalEstimation(objectCloudPtr, objectNormalsPtr, normalRadius_);
	success = recognition_.addModel(*objectCloudPtr, *objectNormalsPtr, objectname);

	if(success)
		ROS_INFO("Loading Model %s: SUCCESS", objectname.c_str());
	else
		ROS_ERROR("Loading Model %s: FAIL", objectname.c_str());
}

void
WmObjectRecognition::normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr srcCloud_o_Ptr, pcl::PointCloud<pcl::Normal>::Ptr normalCloud_o_Ptr, double radius_d)
{
	//create normal estimation object
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation_o;

	//create empty kd tree representation and assign it to the normal estimation object
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree_o_Ptr (new pcl::search::KdTree<pcl::PointXYZ>());
	normalEstimation_o.setSearchMethod(kdTree_o_Ptr);


	//set input cloud
	normalEstimation_o.setInputCloud(srcCloud_o_Ptr);

	//set search radius
	normalEstimation_o.setRadiusSearch(radius_d);

	//compute the normals for the cloud of the current loop
	normalEstimation_o.compute(*normalCloud_o_Ptr);

	//terminal output for some information
	std::cout << "Normals computed." << std::endl;
}

bool
WmObjectRecognition::recognize(pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloudPtr,
		pcl::PointCloud<pcl::Normal>::Ptr sceneNormalsPtr,
		std::list<pcl::recognition::ObjRecRANSAC::Output>& output)
{
	bool successProbability;
	recognition_.recognize(*sceneCloudPtr, *sceneNormalsPtr, output, successProbability);
	return successProbability;
}

} /* namespace wm_object */

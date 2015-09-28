/*
 * mapper.cpp
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */

#include "wm_navigation/WmMapServer.h"

namespace wm_map_server{


WmMapServer::WmMapServer(ros::NodeHandle private_nh_)
: nh_(),
  pointcloudMinZ_(-std::numeric_limits<double>::max()),
  pointcloudMaxZ_(std::numeric_limits<double>::max()),
  res_(0.05),
  do_mapping_(true),
  worldFrameId_("/map"), baseFrameId_("/base_footprint"),
  map_(new pcl::PointCloud<pcl::PointXYZRGB>),
  octree_(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(128.0f)),
  origin_()
{
	ros::NodeHandle private_nh(private_nh_);
	private_nh.param("frame_id", worldFrameId_, worldFrameId_);
	private_nh.param("base_frame_id", baseFrameId_, baseFrameId_);
	private_nh.param("pointcloud_min_z", pointcloudMinZ_,pointcloudMinZ_);
	private_nh.param("pointcloud_max_z", pointcloudMaxZ_,pointcloudMaxZ_);
	private_nh.param("resolution", res_, res_);
	private_nh.param("mapping", do_mapping_, do_mapping_);

	if(do_mapping_)
	{
		pointCloudSub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "cloud_in", 5);
		tfPointCloudSub_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*pointCloudSub_, tfListener_, worldFrameId_, 5);
		tfPointCloudSub_->registerCallback(boost::bind(&WmMapServer::insertCloudCallback, this, _1));
		origin_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000, &WmMapServer::setOriginCallback, this);
	}

	mapService_ = nh_.advertiseService("map", &WmMapServer::mapSrv, this);
	pointCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map", 1, true); //latched

	origin_.position.x = 0.0;
	origin_.position.y = 0.0;
	origin_.position.z = 0.0;
	origin_.orientation.x = 0.0;
	origin_.orientation.y = 0.0;
	origin_.orientation.z = 0.0;
	origin_.orientation.w = 1.0;

	octree_->setInputCloud(map_);

}

WmMapServer::~WmMapServer() {

}

void
WmMapServer::step()
{

	if(!do_mapping_)
	{
		tf::StampedTransform World2Map;

		tf::Quaternion q;

		q.setEuler(0.0f, 0.0f, 0.0);

		World2Map.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		World2Map.setRotation(q);

		World2Map.frame_id_ = "/world";
		World2Map.stamp_ = ros::Time::now();
		World2Map.child_frame_id_ = worldFrameId_;

		try
		{
			tfBroadcaster_.sendTransform(World2Map);

		}catch(tf::TransformException & ex)
		{
				ROS_WARN("WmMapServer::step %s",ex.what());
		}
	}

	publishMap(ros::Time::now());
}

bool
WmMapServer::mapSrv(wm_navigation::GetMap::Request  &req, wm_navigation::GetMap::Response &res)
{

	ROS_INFO("Sending map  on service request");

	if (map_->size() <= 1){
		ROS_WARN("Nothing to publish, map is empty");
		return false;
	}


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	tf::Transform Odom2Map;

	Odom2Map.setOrigin(tf::Vector3(origin_.position.x, origin_.position.y, origin_.position.z));
	Odom2Map.setRotation(tf::Quaternion(origin_.orientation.x, origin_.orientation.y, origin_.orientation.z, origin_.orientation.w));
	Eigen::Matrix4f Odom2Maptf;
	pcl_ros::transformAsMatrix(Odom2Map.inverse(), Odom2Maptf);

	pcl::transformPointCloud(*map_, *map_temp, Odom2Maptf);

	pcl::toROSMsg (*map_temp, res.map);
	res.map.header.frame_id = worldFrameId_;
	res.map.header.stamp = ros::Time::now();

	return true;
}


void
WmMapServer::setOriginCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& origin)
{
	origin_ = origin->pose.pose;
}

void
WmMapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
	ros::WallTime startTime = ros::WallTime::now();

	if(!do_mapping_)
		return;


	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

	pcl_conversions::toPCL(*cloud_in, *cloud);

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (res_, res_, res_);
	sor.filter (*cloud_filtered);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>); // input cloud for filtering and ground-detection
	pcl::fromPCLPointCloud2 (*cloud_filtered, *pc);

	tf::StampedTransform sensorToWorldTf;
	try {
		tfListener_.lookupTransform(worldFrameId_, cloud_in->header.frame_id, cloud_in->header.stamp, sensorToWorldTf);
	} catch(tf::TransformException& ex){
		ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}

	Eigen::Matrix4f sensorToWorld;
	pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(pointcloudMinZ_, pointcloudMaxZ_);

	// directly transform to map frame:
	pcl::transformPointCloud(*pc, *pc, sensorToWorld);

	// just filter height range:
	pass.setInputCloud(pc->makeShared());
	pass.filter(*pc);

	int c=0;

	bool newPoint = false;
	for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it=pc->begin(); it!=pc->end(); ++it)
		if(validNewPoint(*it))
		{
			map_->push_back(*it);
			octree_->addPointToCloud (*it, map_);
			c++;
		}

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Pointcloud insertion in map done (%d new points, %zu pts total, %f sec)", c, map_->size(), total_elapsed);

	publishMap(cloud_in->header.stamp);

	return;
}

bool
WmMapServer::validNewPoint(const pcl::PointXYZRGB& point)
{
	bool ret;

	if(octree_->getInputCloud()->size()<1) return true;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	ret = octree_->radiusSearch(point, res_, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0;

	return ret;

}

bool
WmMapServer::openFile(const std::string& filename)
{

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *map_) == -1)
	{
		ROS_ERROR ("Couldn't read file %s", filename.c_str());
		return false;
	}

	return true;
}

void WmMapServer::publishMap(const ros::Time& rostime){

	ros::WallTime startTime = ros::WallTime::now();

	size_t mapSize = map_->size();

	if (mapSize <= 1){
		ROS_WARN("Nothing to publish, octree is empty");
		return;
	}

	if (pointCloudPub_.getNumSubscribers() > 0){

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
		tf::Transform Odom2Map;

		Odom2Map.setOrigin(tf::Vector3(origin_.position.x, origin_.position.y, origin_.position.z));
		Odom2Map.setRotation(tf::Quaternion(origin_.orientation.x, origin_.orientation.y, origin_.orientation.z, origin_.orientation.w));
		Eigen::Matrix4f Odom2Maptf;
		pcl_ros::transformAsMatrix(Odom2Map.inverse(), Odom2Maptf);

		pcl::transformPointCloud(*map_, *map_temp, Odom2Maptf);

		sensor_msgs::PointCloud2 cloud;
		pcl::toROSMsg (*map_temp, cloud);
		cloud.header.frame_id = worldFrameId_;
		cloud.header.stamp = rostime;
		pointCloudPub_.publish(cloud);
	}

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);

}

}

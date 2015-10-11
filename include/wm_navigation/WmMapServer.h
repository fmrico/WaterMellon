/*
 * mapper.h
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */

#ifndef WMMAPSERVER_H_
#define WMMAPSERVER_H_

#include "ros/ros.h"
#include <ros/console.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree.h>

#include <wm_navigation/GetMap.h>

namespace wm_map_server {

class WmMapServer {
public:

	WmMapServer(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
	virtual ~WmMapServer();

	void step();

	void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
	void setOriginCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& origin);
	bool mapSrv(wm_navigation::GetMap::Request  &req, wm_navigation::GetMap::Response &res);

	virtual bool openFile(const std::string& filename);

private:

	void publishMap(const ros::Time& rostime = ros::Time::now());
	bool validNewPoint(const pcl::PointXYZRGB& point);

	ros::NodeHandle nh_;
	ros::Publisher pointCloudPub_;
	tf::TransformListener tfListener_;
	tf::TransformBroadcaster tfBroadcaster_;
	tf::MessageFilter<sensor_msgs::PointCloud2>* tfPointCloudSub_;

	message_filters::Subscriber<sensor_msgs::PointCloud2>* pointCloudSub_;
	ros::ServiceServer mapService_;
	ros::Subscriber origin_sub_;

	geometry_msgs::Pose origin_;

	std::string worldFrameId_;
	std::string baseFrameId_;

	double pointcloudMinZ_;
	double pointcloudMaxZ_;
	double pointcloudMaxX_;
	double pointcloudMaxTxy_;
	double pointcloudMaxTxz_;
	double res_;
	bool do_mapping_;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr octree_;

};

}

#endif /* WMMAPSERVER_H_ */

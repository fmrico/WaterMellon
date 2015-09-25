/*
 * mapper.h
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */

#ifndef WMMAPSERVER_H_
#define WMMAPSERVER_H_

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

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
//#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree.h>

#include <wm_navigation/GetMap.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/console.h>

namespace wm_map_server {

class WmMapServer {
public:

	WmMapServer(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
	virtual ~WmMapServer();

	virtual void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
	virtual bool mapSrv(wm_navigation::GetMap::Request  &req, wm_navigation::GetMap::Response &res);

	virtual bool openFile(const std::string& filename);

protected:

		void publishMap(const ros::Time& rostime = ros::Time::now());
		bool validNewPoint(const pcl::PointXYZRGB& point);

	  ros::NodeHandle m_nh;
		ros::Publisher m_pointCloudPub;
	  tf::TransformListener m_tfListener;
	  tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;

	  message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
  	ros::ServiceServer m_mapService;

	  std::string m_worldFrameId;
	  std::string m_baseFrameId;

	  double m_pointcloudMinZ;
	  double m_pointcloudMaxZ;
	  double m_res;
	  bool m_mapping;

	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr map;
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr octree;
 
};

}

#endif /* WMMAPSERVER_H_ */

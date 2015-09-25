/*
 * wm_imarkers.hpp
 *
 *  Created on: 05/09/2015
 *      Author: paco
 */

#ifndef WM_IMARKERS_HPP_
#define WM_IMARKERS_HPP_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

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

#include <sensor_msgs/PointCloud2.h>

namespace wm_markers {

class WmIMarkers {
public:
	WmIMarkers();
	~WmIMarkers();

	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

	virtual void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);

protected:

	void initMarker(visualization_msgs::InteractiveMarker &marker,  std::string name, std::string desc, geometry_msgs::Pose pose);
	void publishObject(const ros::Time& rostime = ros::Time::now());

	interactive_markers::InteractiveMarkerServer server_;

	visualization_msgs::InteractiveMarker max_marker_;
	visualization_msgs::InteractiveMarker min_marker_;

	std::string frame_id_;

	tf::TransformListener m_tfListener;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_;

	ros::NodeHandle m_nh;
	ros::Publisher ObjectPub_;
	tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;
	message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;

};

}

#endif /* WM_IMARKERS_HPP_ */

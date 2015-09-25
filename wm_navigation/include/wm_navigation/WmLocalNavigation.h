/*
 * mapper.h
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */

#ifndef WMLOCALNAVIGATION_H_
#define WMLOCALNAVIGATION_H_

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>


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
#include "pcl/point_types_conversion.h"
//#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree.h>

#include <wm_navigation/GetMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ros/console.h>

#include <float.h>
#include <boost/math/distributions.hpp>
#include <boost/math/distributions/normal.hpp>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <time.h>
//#include <random>

#include <wm_navigation/Particle.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

namespace wm_navigation {


class WmLocalNavigation {
public:

	WmLocalNavigation(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
	virtual ~WmLocalNavigation();

	virtual void gvectorCallback(const geometry_msgs::PoseStamped::ConstPtr& gvector_in);
	virtual void perceptionCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);

	virtual void step();

private:

	void publish_all();
	void publish_repulsive_vector();
	void publish_resultant_vector();

	ros::NodeHandle m_nh;
	tf::TransformListener m_tfListener;
	tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPerceptSub;
	message_filters::Subscriber<sensor_msgs::PointCloud2>* m_perceptSub;

	ros::Subscriber goal_sub;
	ros::Publisher vel_pub;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_perception;

	double m_pointcloudMinZ;
	double m_pointcloudMaxZ;
	double m_res;
	double robot_radious;
	double collision_area;
	double max_w;
	double max_v;

	std::string m_worldFrameId;
	std::string m_baseFrameId;

	geometry_msgs::PoseStamped::Ptr global_vector;
	geometry_msgs::PoseStamped::Ptr resultant_vector;
	geometry_msgs::PoseStamped::Ptr repulsive_vector;


	ros::Publisher repulsive_vector_pub;
	ros::Publisher resultant_vector_pub;

	bool has_goal;
};

}

#endif /* WMGLOBALNAVIGATION_H_ */

/*
 * mapper.h
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */

#ifndef WMGLOBALNAVIGATION_H_
#define WMGLOBALNAVIGATION_H_

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

namespace wm_navigation {


class WmGlobalNavigation {
public:

	WmGlobalNavigation(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
	virtual ~WmGlobalNavigation();

	virtual void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& map_in);
	virtual void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_in);
	virtual void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& goal_in);

	virtual void step();

private:

	void publish_all();
	void publish_map();
	void publish_gradient();
	void publish_goal();

	void updateCostmap();
	void updateGradient(int i, int j, int cost);

	inline void xy2ij(const double x, const double y, int &i, int &j) {
		i = (x - goal_gradient.getOriginX()) / goal_gradient.getResolution();
		j = (y - goal_gradient.getOriginY()) / goal_gradient.getResolution();
	}

	inline void ij2xy(const int i, const int j, double &x, double &y) {
		x = costmap.getOriginX() + i*costmap.getResolution() + costmap.getResolution()/2;
		y = costmap.getOriginY() + j*costmap.getResolution() + costmap.getResolution()/2;
	}
	inline void ij2xy(const int i, const int j, float &x, float &y) {
		x = costmap.getOriginX() + i*costmap.getResolution() + costmap.getResolution()/2;
		y = costmap.getOriginY() + j*costmap.getResolution() + costmap.getResolution()/2;
	}
	ros::NodeHandle m_nh;

	tf::TransformListener m_tfListener;
	tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfMapSub;
	message_filters::Subscriber<sensor_msgs::PointCloud2>* m_mapSub;
	ros::Subscriber goal_sub;
	ros::Subscriber pose_sub;

	std::string m_worldFrameId;
	std::string m_baseFrameId;

	geometry_msgs::PoseWithCovarianceStamped pose;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr map;

	float map_max_x, map_min_x, map_max_y, map_min_y;


	costmap_2d::Costmap2D costmap;
	costmap_2d::Costmap2D goal_gradient;
	costmap_2d::Costmap2DPublisher costmap_pub;
	costmap_2d::Costmap2DPublisher goal_gradient_pub;


	std::vector<geometry_msgs::PoseStamped> plan;
	geometry_msgs::PoseStamped::Ptr goal;
	geometry_msgs::PoseStamped::Ptr goal_vector;

	ros::Publisher goal_vector_pub;

	bool has_goal;
};

}

#endif /* WMGLOBALNAVIGATION_H_ */

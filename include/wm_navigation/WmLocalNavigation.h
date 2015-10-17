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
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

#include <wm_navigation/Utilities.h>
#include <watermellon/GNavGoalStamped.h>

namespace wm_navigation {


class WmLocalNavigation {
public:

	WmLocalNavigation(ros::NodeHandle private_nh = ros::NodeHandle("~"));
	virtual ~WmLocalNavigation();

	virtual void gvectorCallback(const watermellon::GNavGoalStamped::ConstPtr& gvector_in);
	virtual void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);

	virtual void step();

private:

	void publish_all();
	void publish_repulsive_vector();
	void publish_resultant_vector();
	void publish_atractive_vector();

	void addVectors(geometry_msgs::Twist& v1, const geometry_msgs::Twist& v2);
	ros::NodeHandle nh_;
	tf::TransformListener tfListener_;

	tf::MessageFilter<sensor_msgs::LaserScan>* tfScanSub_;
	message_filters::Subscriber<sensor_msgs::LaserScan>* scanSub_;

	ros::Subscriber goal_sub_;
	ros::Publisher vel_pub_;

	double robot_radious_;
	double collision_area_;
	double max_w_;
	double max_v_;

	std::string worldFrameId_;
	std::string baseFrameId_;

	watermellon::GNavGoalStamped::Ptr goal_;

	geometry_msgs::TwistStamped::Ptr repulsive_vector_;
	geometry_msgs::TwistStamped::Ptr resultant_vector_;
	geometry_msgs::TwistStamped::Ptr atractive_vector_;


	ros::Publisher atractive_vector_pub_;
	ros::Publisher repulsive_vector_pub_;
	ros::Publisher resultant_vector_pub_;

	static const int FAR = 0;
	static const int NEAR = 1;
	static const int TURNING = 2;
	static const int FINISHED = 3;

	int state_;

};

}

#endif /* WMGLOBALNAVIGATION_H_ */

/*
 * mapper.h
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */

#ifndef WMLOCALIZATION_H_
#define WMLOCALIZATION_H_

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <pcl/point_types.h>
//#include <pcl/conversions.h>
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

#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree.h>

#include <wm_navigation/GetMap.h>
#include <sensor_msgs/PointCloud2.h>

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
#include <wm_navigation/Utilities.h>
namespace wm_localization {


class WmLocalization {
public:

	WmLocalization(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
	~WmLocalization();

	void perceptionCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
	void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& map_in);
	void setPosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pos);


	void step();

private:

	void predict();
	void correct();

	void resetParticles();
	void resetParticleUniform(Particle& p);
	void resetParticleNear(Particle& p2reset, const Particle& pref);

	void setInitPose(const geometry_msgs::Pose& init_coord);

	void updateParticle(Particle& p, const std::vector<pcl::PointXYZRGB>& testpoints);
	float doTestPcl(pcl::PointXYZRGB &searchPoint);
	void updatePos();
	void reseed();
	void normalize();

	void publishAll();
	void publishPerception(const ros::Time& rostime = ros::Time::now());
	void publishParticles(const ros::Time& rostime = ros::Time::now());
	void publishPose(const ros::Time& rostime = ros::Time::now());
	void publishPoseWithCov(const ros::Time& rostime = ros::Time::now());

	bool validParticlePosition(float x, float y);


	/*******************************************************
	/* Test  to remove
	/*******************************************************/
	void setParticle(Particle &particle, float x, float y, float theta);
	ros::Publisher testpoints_pub;

	/*******************************************************
	/* Test functions to remove end
	/*******************************************************/

	ros::NodeHandle nh_;

	tf::TransformListener tfListener_;
	tf::MessageFilter<sensor_msgs::PointCloud2>* tfPerceptSub_;
	tf::MessageFilter<sensor_msgs::PointCloud2>* tfMapSub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2>* perceptSub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2>* mapSub_;

	ros::Publisher perception_pub_;
	ros::Publisher particles_pub_;
	ros::Publisher pose_pub_;
	ros::Publisher posecov_pub_;
	ros::Subscriber pose_sub_;

	int numparticles_min_;
	int numparticles_max_;

	int correctpoints_;
	double res_;
	double odomerror_;
	double pointcloudMinZ_;
	double pointcloudMaxZ_;

	bool doResetParticles_;

	std::string worldFrameId_;
	std::string baseFrameId_;

	std::vector<Particle> particles_;
	geometry_msgs::PoseWithCovariance pose_;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_perception_;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr map_;

	float map_max_x_, map_min_x_, map_max_y_, map_min_y_;

	//std::default_random_engine generator;
	//std::normal_distribution<float> *distribution;

	tf::Transform last2odom_;
	tf::Transform genNoise(tf::Transform &base);

};

}

#endif /* WMLOCALIZATION_H_ */

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
//#include <pcl/kdtree/kdtree_flann.h>

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

namespace wm_localization {

inline double
randn (double mu, double sigma)
{
  double U1, U2, W, mult;
  static double X1, X2;
  static int call = 0;

  if (call == 1)
    {
      call = !call;
      return (mu + sigma * (double) X2);
    }

  do
    {
      U1 = -1 + ((double) rand () / RAND_MAX) * 2;
      U2 = -1 + ((double) rand () / RAND_MAX) * 2;
      W = pow (U1, 2) + pow (U2, 2);
    }
  while (W >= 1 || W == 0);

  mult = sqrt ((-2 * log (W)) / W);
  X1 = U1 * mult;
  X2 = U2 * mult;

  call = !call;

  return (mu + sigma * (double) X1);
}

inline double toRadians(double degrees){return degrees * (M_PI/180.0);}

inline double normalizePi(double data)
{
  if (data < M_PI && data >= -M_PI) return data;
  double ndata = data - ((int )(data / (M_PI*2.0)))*(M_PI*2.0);
  while (ndata >= M_PI)
  {
    ndata -= (M_PI*2.0);
  }
  while (ndata < -M_PI)
  {
    ndata += (M_PI*2.0);
  }
  return ndata;
}

class WmLocalization {
public:

	WmLocalization(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
	virtual ~WmLocalization();

	virtual void perceptionCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
	virtual void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& map_in);
//	virtual void odomCB(const nav_msgs::Odometry::ConstPtr& msg);

	virtual void step();

protected:

	virtual void predict();
	virtual void correct();

	virtual void resetParticles();
	virtual void resetParticleUniform(Particle& p);
	virtual void resetParticleNear(Particle& p2reset, const Particle& pref);

	virtual void updateParticle(Particle& p, const std::vector<pcl::PointXYZRGB>& testpoints);
	virtual float doTestPcl(pcl::PointXYZRGB &searchPoint);
	virtual void updatePos();
	virtual void reseed();
	virtual void normalize();

	virtual void publishAll();
	virtual void publishPerception(const ros::Time& rostime = ros::Time::now());
	virtual void publishParticles(const ros::Time& rostime = ros::Time::now());
	virtual void publishPose(const ros::Time& rostime = ros::Time::now());
	virtual void publishPoseWithCov(const ros::Time& rostime = ros::Time::now());

	bool validParticlePosition(float x, float y);


	/*******************************************************
	/* Test  to remove
	/*******************************************************/
	void setParticle(Particle &particle, float x, float y, float theta);
	ros::Publisher testpoints_pub;

	/*******************************************************
	/* Test functions to remove end
	/*******************************************************/

	ros::NodeHandle m_nh;

	tf::TransformListener m_tfListener;
	tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPerceptSub;
	tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfMapSub;
	message_filters::Subscriber<sensor_msgs::PointCloud2>* m_perceptSub;
	message_filters::Subscriber<sensor_msgs::PointCloud2>* m_mapSub;

	ros::Publisher perception_pub;
	ros::Publisher particles_pub;
	ros::Publisher pose_pub;
	ros::Publisher posecov_pub;

	int m_numparticles_min;
	int m_numparticles_max;

	int m_correctpoints;
	double m_res;
	double m_odomerror;
	double m_pointcloudMinZ;
	double m_pointcloudMaxZ;

	bool doResetParticles;

	std::string m_worldFrameId;
	std::string m_baseFrameId;

	std::vector<Particle> particles;
	geometry_msgs::PoseWithCovariance pose;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_perception;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr map;

	float map_max_x, map_min_x, map_max_y, map_min_y;

	//std::default_random_engine generator;
	//std::normal_distribution<float> *distribution;

	tf::Transform last2odom;
	tf::Transform genNoise(tf::Transform &base);

};

}

#endif /* WMLOCALIZATION_H_ */

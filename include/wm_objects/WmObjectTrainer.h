/*
 * WmObjectTrainer.h
 *
 *  Created on: 19/10/2015
 *      Author: paco
 */

#ifndef WMOBJECTTRAINER_H_
#define WMOBJECTTRAINER_H_

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <watermellon/GetObject.h>
#include <combinations.h>

namespace wm_objects {


class WmObjectTrainer {
public:
	WmObjectTrainer(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
	virtual ~WmObjectTrainer();

	void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
	bool objectSrv(watermellon::GetObject::Request  &req, watermellon::GetObject::Response &res);

private:

	bool updateObjectFrame(const ros::Time& stamp, tf::StampedTransform& m2c);
	void initMarkers();
	bool validNewPoint(const pcl::PointXYZRGB& point);

	void calculateSetMeanStdv(const std::vector<tf::StampedTransform>&set, tf::Vector3& mean, tf::Vector3& stdev);
	void getValidMarks(std::vector<tf::StampedTransform>& marks, const ros::Time& stamp);
	void getBestTransform(const std::vector<tf::StampedTransform>& marks, tf::Transform& trans, double& stdev);
	static const int MAX_MARKS = 9;
	static const int MIN_VALID_MARKS = 4;

	ros::NodeHandle nh_;

	tf::TransformBroadcaster tfBroadcaster_;
	tf::TransformListener tfListener_;

	tf::MessageFilter<sensor_msgs::PointCloud2>* tfPointCloudSub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2>* pointCloudSub_;

	double max_z_;
	std::string cameraFrameId_;
	std::string objectFrameId_;
	std::string cameraTopicId_;

	std::map<std::string, tf::Transform> marks2center_;
	tf::Transform alt_marks2center_[MAX_MARKS];

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr object_octree_;

	ros::Publisher objectPub_;
	ros::ServiceServer objectService_;
};

}

#endif /* WMOBJECTTRAINER_H_ */

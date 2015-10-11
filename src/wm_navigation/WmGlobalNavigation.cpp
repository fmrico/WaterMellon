/*
 * mapper.cpp
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */

#include "wm_navigation/WmGlobalNavigation.h"

namespace wm_navigation{


WmGlobalNavigation::WmGlobalNavigation(ros::NodeHandle private_nh_)
: nh_(),
  pointcloudMinZ_(0.2),
  pointcloudMaxZ_(0.5),
  worldFrameId_("/map"), baseFrameId_("base_footprint"),
  map_(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(128.0f)),
  map_max_x_(1.0), map_min_x_(0.0), map_max_y_(1.0), map_min_y_(0.0),
  res_(0.05),
  dynamic_cost_dec_(5.0),
  dynamic_cost_inc_(20.0),
  tfListener_(),
  static_costmap_(),
  dynamic_costmap_(),
  goal_gradient_(),
  static_costmap_pub_(&nh_, &static_costmap_, "/map", "/wm_navigation_static_costmap", true),
  dynamic_costmap_pub_(&nh_, &dynamic_costmap_, "/map", "/wm_navigation_dynamic_costmap", true),
  goal_gradient_pub_(&nh_, &goal_gradient_, "/map", "/wm_navigation_goal_gradient", true),
  goal_vector_(new geometry_msgs::TwistStamped),
  goal_(new geometry_msgs::PoseStamped),
  start_(new geometry_msgs::Pose),
  last_perception_(new pcl::PointCloud<pcl::PointXYZRGB>),
  has_goal_(false)
 {
	ros::NodeHandle private_nh(private_nh_);
	private_nh.param("frame_id", worldFrameId_, worldFrameId_);
	private_nh.param("base_frame_id", baseFrameId_, baseFrameId_);
	private_nh.param("resolution", res_, res_);
	private_nh.param("pointcloud_min_z", pointcloudMinZ_,pointcloudMinZ_);
	private_nh.param("pointcloud_max_z", pointcloudMaxZ_,pointcloudMaxZ_);
	private_nh.param("dynamic_cost_dec", dynamic_cost_dec_,dynamic_cost_dec_);
	private_nh.param("dynamic_cost_inc", dynamic_cost_inc_,dynamic_cost_inc_);

	perceptSub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "cloud_in", 5);
	tfPerceptSub_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*perceptSub_, tfListener_, baseFrameId_, 5);
	tfPerceptSub_->registerCallback(boost::bind(&WmGlobalNavigation::perceptionCallback, this, _1));

	mapSub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "map", 5);
	tfMapSub_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*mapSub_, tfListener_, worldFrameId_, 5);
	tfMapSub_->registerCallback(boost::bind(&WmGlobalNavigation::mapCallback, this, _1));

	goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &WmGlobalNavigation::goalCallback, this);
	pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/wm_pose_cov", 1, &WmGlobalNavigation::poseCallback, this);
	goal_vector_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/goal_vector", 1000);


	last_dynamic_map_update_ = ros::WallTime::now();



}

WmGlobalNavigation::~WmGlobalNavigation() {

}

void
WmGlobalNavigation::perceptionCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
	ros::WallTime startTime = ros::WallTime::now();

	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

	pcl_conversions::toPCL(*cloud_in, *cloud);

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (dynamic_costmap_.getResolution(),
			dynamic_costmap_.getResolution(),
			dynamic_costmap_.getResolution());
	sor.filter (*cloud_filtered);

	pcl::fromPCLPointCloud2 (*cloud_filtered, *last_perception_);

	tf::StampedTransform sensorToBaseTf;
	try {
		tfListener_.lookupTransform(baseFrameId_, cloud_in->header.frame_id, cloud_in->header.stamp, sensorToBaseTf);
	} catch(tf::TransformException& ex){
		ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}

	Eigen::Matrix4f sensorToBase;
	pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);

	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(pointcloudMinZ_, pointcloudMaxZ_);

	// directly transform to map frame:
	pcl::transformPointCloud(*last_perception_, *last_perception_, sensorToBase);

	// just filter height range:
	pass.setInputCloud(last_perception_->makeShared());
	pass.filter(*last_perception_);

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	//ROS_DEBUG("Pointcloud perception done (%zu pts total, %f sec)", last_perception_->size(), total_elapsed);

	return;
}

void
WmGlobalNavigation::setGoalPose(const geometry_msgs::PoseStamped& goal_in)
{
	*goal_ = goal_in;
	*start_ = pose_.pose.pose;
	has_goal_ = true;


}
geometry_msgs::Pose
WmGlobalNavigation::getCurrentPose()
{
	return pose_.pose.pose;
}

geometry_msgs::Pose
WmGlobalNavigation::getStartingPose()
{
	return *start_;
}

geometry_msgs::Pose
WmGlobalNavigation::getEndPose()
{
	return goal_->pose;
}


void
WmGlobalNavigation::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_in)
{

	pose_ = *pose_in;
	ROS_DEBUG("Pose received (%f, %f)",  pose_.pose.pose.position.x, pose_.pose.pose.position.y);
}

void
WmGlobalNavigation::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_in)
{
	setGoalPose(*goal_in);
}


void
WmGlobalNavigation::mapCallback(const sensor_msgs::PointCloud2::ConstPtr& map_in)
{

	ros::WallTime startTime = ros::WallTime::now();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_in_xyz (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());

	pcl_conversions::toPCL(*map_in, *cloud);
	pcl::fromPCLPointCloud2 (*cloud, *map_in_xyz);

	map_->setInputCloud (map_in_xyz);
	map_->addPointsFromInputCloud ();


	ROS_DEBUG("Map Pointcloud %zu",  map_in_xyz->size());

	if(!map_in_xyz->empty())
	{
		ROS_INFO("Map Received; Unsubscribing from mapserver");
		mapSub_->unsubscribe();
	}
	pcl::PointCloud<pcl::PointXYZRGB>::iterator it;

	map_max_x_ = map_max_y_ =FLT_MIN;
	map_min_x_ = map_min_y_ =FLT_MIN;


	for(it=map_in_xyz->begin(); it!=map_in_xyz->end(); ++it)
	{
		if(it->x < map_min_x_) map_min_x_ = it->x;
		if(it->y < map_min_y_) map_min_y_ = it->y;
		if(it->x > map_max_x_) map_max_x_ = it->x;
		if(it->y > map_max_y_) map_max_y_ = it->y;
	}

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Map reception [ %f -> %f, %f -> %f ]done (%zu pts total, %f sec)",
			map_min_x_, map_max_x_, map_min_y_, map_max_y_, map_in_xyz->size(), total_elapsed);

	static_costmap_.resizeMap((map_max_x_-map_min_x_)*10, (map_max_y_-map_min_y_)*10, 0.1, map_min_x_, map_min_y_);
	dynamic_costmap_.resizeMap((map_max_x_-map_min_x_)*10, (map_max_y_-map_min_y_)*10, 0.1, map_min_x_, map_min_y_);
	goal_gradient_.resizeMap((map_max_x_-map_min_x_)*10, (map_max_y_-map_min_y_)*10, 0.1, map_min_x_, map_min_y_);

	static_costmap_.setDefaultValue(255);
	static_costmap_.resetMap(0,0,static_costmap_.getSizeInCellsX(), static_costmap_.getSizeInCellsY());

	goal_gradient_.setDefaultValue(255);
	goal_gradient_.resetMap(0,0,goal_gradient_.getSizeInCellsX(), goal_gradient_.getSizeInCellsY());

	dynamic_costmap_.setDefaultValue(0);
	dynamic_costmap_.resetMap(0,0,dynamic_costmap_.getSizeInCellsX(), dynamic_costmap_.getSizeInCellsY());
	updateStaticCostmap();

	return;
}

void
WmGlobalNavigation::updateDynamicCostmap()
{
	double total_elapsed = (ros::WallTime::now() - last_dynamic_map_update_).toSec();

	for(int i=0; i<dynamic_costmap_.getSizeInCellsX(); i++)
		for(int j=0; j<dynamic_costmap_.getSizeInCellsY(); j++)
		{
			double new_cost;
			new_cost = dynamic_costmap_.getCost(i, j) - (dynamic_cost_dec_*total_elapsed);
			if(new_cost < 0) new_cost=0;

			if(i==dynamic_costmap_.getSizeInCellsX()-1 && j==dynamic_costmap_.getSizeInCellsY()-1)
				ROS_DEBUG("(%d, %lf)", dynamic_costmap_.getCost(i, j), new_cost);

			dynamic_costmap_.setCost(i, j, new_cost); //10 unids/sec
	}


	tf::StampedTransform robot2MapTf;
	robot2MapTf.frame_id_=baseFrameId_;
	robot2MapTf.child_frame_id_=worldFrameId_;
	robot2MapTf.stamp_=ros::Time::now();
	robot2MapTf.setOrigin(tf::Vector3(pose_.pose.pose.position.x, pose_.pose.pose.position.y,
			pose_.pose.pose.position.z));
	robot2MapTf.setRotation(tf::Quaternion(pose_.pose.pose.orientation.x, pose_.pose.pose.orientation.y,
			pose_.pose.pose.orientation.z, pose_.pose.pose.orientation.w ));

	Eigen::Matrix4f robot2Map;
	pcl_ros::transformAsMatrix(robot2MapTf, robot2Map);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_perception_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*last_perception_, *last_perception_temp, robot2Map);


	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr perception_octree(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(128.0f));

	perception_octree->setInputCloud (last_perception_temp);
	perception_octree->addPointsFromInputCloud ();

	for(int i=0; i<dynamic_costmap_.getSizeInCellsX(); i++)
		for(int j=0; j<dynamic_costmap_.getSizeInCellsY(); j++)
		{
			pcl::PointXYZRGB gridPoint;

			ij2xy(dynamic_costmap_, i,j, gridPoint.x, gridPoint.y);

			gridPoint.z = 0.5;

			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			int npoints;

			npoints = perception_octree->radiusSearch(gridPoint, 0.3, pointIdxRadiusSearch, pointRadiusSquaredDistance);

			if(npoints>0)
			{


				double new_cost;
				new_cost =  dynamic_costmap_.getCost(i, j) + (dynamic_cost_inc_*total_elapsed);
				if(new_cost>255) new_cost=255;

				dynamic_costmap_.setCost(i, j, new_cost);
			}

		}


	last_dynamic_map_update_ = ros::WallTime::now();
}

void
WmGlobalNavigation::updateStaticCostmap()
{

	for(int i=0; i<static_costmap_.getSizeInCellsX(); i++)
		for(int j=0; j<static_costmap_.getSizeInCellsY(); j++)
		{
			pcl::PointXYZRGB gridPoint;

			ij2xy(static_costmap_, i,j, gridPoint.x, gridPoint.y);

			gridPoint.z = 0.5;

			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			int npoints;

			npoints = map_->radiusSearch(gridPoint, 0.3, pointIdxRadiusSearch, pointRadiusSquaredDistance);

			if(npoints==0)
				static_costmap_.setCost(i, j, 0);
			else
			{
				std::sort(pointRadiusSquaredDistance.begin(), pointRadiusSquaredDistance.end());
				static_costmap_.setCost(i, j, (1.0-(pointRadiusSquaredDistance[0]/0.3)*255));
			}

		}
}

void
WmGlobalNavigation::updateGradient(int i, int j, int cost)
{
	if(i<0) return;
	if(i>(goal_gradient_.getSizeInCellsX()-1)) return;
	if(j<0) return;
	if(j>(goal_gradient_.getSizeInCellsY()-1)) return;

	if(static_costmap_.getCost(i,j)>0) return;
	if(dynamic_costmap_.getCost(i,j)>0) return;

	if(goal_gradient_.getCost(i,j)<=cost)
		return;

	goal_gradient_.setCost(i,j, cost);

	updateGradient(i+1, j, cost+1);
	updateGradient(i-1, j, cost+1);
	updateGradient(i, j+1, cost+1);
	updateGradient(i, j-1, cost+1);

	updateGradient(i+1, j+1, cost+1);
	updateGradient(i-1, j+1, cost+1);
	updateGradient(i+1, j-1, cost+1);
	updateGradient(i-1, j-1, cost+1);

	return;
}

void 
WmGlobalNavigation::updatePath()
{
	ros::WallTime startTime = ros::WallTime::now();

	goal_gradient_.resetMap(0,0,goal_gradient_.getSizeInCellsX(), goal_gradient_.getSizeInCellsY());

	int i, j;

	xy2ij(goal_gradient_, goal_->pose.position.x, goal_->pose.position.y, i, j);

	updateGradient(i, j, 0);

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("UpdatePath done (%f sec)", total_elapsed);

}

void
WmGlobalNavigation::step()
{

	ros::WallTime startTime = ros::WallTime::now();
	if((map_->getInputCloud()==NULL)||(map_->getInputCloud()->empty()))
		return;

	updateDynamicCostmap();


	if(has_goal_)
	{
		updatePath();

		int robot_i, robot_j;
		xy2ij(goal_gradient_, pose_.pose.pose.position.x, pose_.pose.pose.position.y, robot_i, robot_j);

		int min;
		min = goal_gradient_.getCost(robot_i, robot_j);


		std::vector<int> menores_i;
		std::vector<int> menores_j;
		for(int i=-3; i<=3; i++)
			for(int j=-4; j<=4; j++)
			{
				if(goal_gradient_.getCost(robot_i+i, robot_j+j)<min)
				{
					menores_i.clear();
					menores_j.clear();
					min = goal_gradient_.getCost(robot_i+i, robot_j+j);
					menores_i.push_back(robot_i+i);
					menores_j.push_back(robot_j+j);
				}
				if(goal_gradient_.getCost(robot_i+i, robot_j+j)==min)
				{
					menores_i.push_back(robot_i+i);
					menores_j.push_back(robot_j+j);
				}
			}

		double g_x, g_y;
		g_x = g_y = 0.0;


		double dist2goal;
		double robot_x, robot_y, goal_x, goal_y;
		robot_x = pose_.pose.pose.position.x;
		robot_y = pose_.pose.pose.position.y;
		goal_x = goal_->pose.position.x;
		goal_y = goal_->pose.position.y;

		dist2goal = sqrt((robot_x-goal_x)*(robot_x-goal_x)+(robot_y-goal_y)*(robot_y-goal_y));

		ROS_DEBUG("DISTANCE2GOAL = %lf", dist2goal);
		if(dist2goal<0.05)
		{
			goal_vector_->twist.linear.x = 0.0;

			double r, p, angle2goal;
			tf::Quaternion q1(goal_->pose.orientation.x, goal_->pose.orientation.y, goal_->pose.orientation.z, goal_->pose.orientation.w);
			tf::Matrix3x3(q1).getRPY(r, p, angle2goal);

			double robot_angle;
			tf::Quaternion q2(pose_.pose.pose.orientation.x, pose_.pose.pose.orientation.y, pose_.pose.pose.orientation.z, pose_.pose.pose.orientation.w);
			tf::Matrix3x3(q2).getRPY(r, p, robot_angle);

			goal_vector_->twist.angular.z = normalizePi(angle2goal-robot_angle);

			ROS_DEBUG("(%lf - %lf -> %lf", angle2goal, robot_angle,  normalizePi(angle2goal-robot_angle));

			/*if(fabs(normalizePi(angle2goal-robot_angle))<0.1)
			{
				goal_vector_->twist.linear.x = 0.0;
				goal_vector_->twist.angular.z = 0.0;
				has_goal_=false;
			}*/

		}else
		{
			for(int i=0; i<menores_i.size();++i)
			{
				double t_x, t_y;
				ij2xy(static_costmap_, menores_i[i], menores_j[i], t_x, t_y);

				g_x = g_x + (t_x-g_x)/(i+1);
				g_y = g_y + (t_y-g_y)/(i+1);
			}

			goal_vector_->twist.linear.x = dist2goal/5.0;


			double angle2goal;
			angle2goal = atan2(g_y - pose_.pose.pose.position.y, g_x - pose_.pose.pose.position.x);

			double r, p, robot_angle;
			tf::Quaternion q(pose_.pose.pose.orientation.x, pose_.pose.pose.orientation.y, pose_.pose.pose.orientation.z, pose_.pose.pose.orientation.w);
			tf::Matrix3x3(q).getRPY(r, p, robot_angle);

			goal_vector_->twist.angular.z = normalizePi(angle2goal-robot_angle);

			ROS_DEBUG("GLOBAL VECTOR = (%lf, %lf)", goal_vector_->twist.linear.x, goal_vector_->twist.angular.z);
		}




	}else{
		goal_vector_->twist.linear.x = 0.0;
		goal_vector_->twist.angular.z = 0.0;

	}


	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("GlobalNavigation done (%f sec)", total_elapsed);
	publish_all();


}

void
WmGlobalNavigation::publish_all()
{
	publish_static_map();
	publish_dynamic_map();
	publish_gradient();
	if(has_goal_) publish_goal();
}

void
WmGlobalNavigation::publish_static_map()
{
	static_costmap_pub_.publishCostmap();
}
void
WmGlobalNavigation::publish_dynamic_map()
{
	dynamic_costmap_pub_.publishCostmap();
}


void
WmGlobalNavigation::publish_gradient()
{
	goal_gradient_pub_.publishCostmap();
}

void
WmGlobalNavigation::publish_goal()
{
	goal_vector_->header.frame_id = baseFrameId_;
	goal_vector_->header.stamp = ros::Time::now();

	goal_vector_pub_.publish(goal_vector_);

}

}
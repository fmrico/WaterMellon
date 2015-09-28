/*
 * mapper.cpp
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */

#include "wm_navigation/WmGlobalNavigation.h"

namespace wm_navigation{


WmGlobalNavigation::WmGlobalNavigation(ros::NodeHandle private_nh_)
: m_nh(),
  pointcloudMinZ_(0.2),
  pointcloudMaxZ_(0.5),
  m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
  map(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(128.0f)),
  map_max_x(1.0), map_min_x(0.0), map_max_y(1.0), map_min_y(0.0),
  res_(0.05),
  dynamic_cost_dec(5.0),
  dynamic_cost_inc(20.0),
  m_tfListener(),
  static_costmap(),
  dynamic_costmap(),
  goal_gradient(),
  static_costmap_pub(&m_nh, &static_costmap, "/map", "/wm_navigation_static_costmap", true),
  dynamic_costmap_pub(&m_nh, &dynamic_costmap, "/map", "/wm_navigation_dynamic_costmap", true),
  goal_gradient_pub(&m_nh, &goal_gradient, "/map", "/wm_navigation_goal_gradient", true),
  goal_vector(new geometry_msgs::TwistStamped),
  goal(new geometry_msgs::PoseStamped),
  last_perception_(new pcl::PointCloud<pcl::PointXYZRGB>),
  has_goal(false)
{
	ros::NodeHandle private_nh(private_nh_);
	private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
	private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
	private_nh.param("resolution", res_, res_);
	private_nh.param("pointcloud_min_z", pointcloudMinZ_,pointcloudMinZ_);
	private_nh.param("pointcloud_max_z", pointcloudMaxZ_,pointcloudMaxZ_);
	private_nh.param("dynamic_cost_dec", dynamic_cost_dec,dynamic_cost_dec);
	private_nh.param("dynamic_cost_inc", dynamic_cost_inc,dynamic_cost_inc);

	perceptSub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
	tfPerceptSub_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*perceptSub_, m_tfListener, m_baseFrameId, 5);
	tfPerceptSub_->registerCallback(boost::bind(&WmGlobalNavigation::perceptionCallback, this, _1));

	m_mapSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "map", 5);
	m_tfMapSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_mapSub, m_tfListener, m_worldFrameId, 5);
	m_tfMapSub->registerCallback(boost::bind(&WmGlobalNavigation::mapCallback, this, _1));

	goal_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &WmGlobalNavigation::goalCallback, this);
	pose_sub = m_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/wm_pose_cov", 1, &WmGlobalNavigation::poseCallback, this);
	goal_vector_pub = m_nh.advertise<geometry_msgs::TwistStamped>("/goal_vector", 1000);


	last_dynamic_map_update = ros::WallTime::now();

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
	sor.setLeafSize (dynamic_costmap.getResolution(),
			dynamic_costmap.getResolution(),
			dynamic_costmap.getResolution());
	sor.filter (*cloud_filtered);

	pcl::fromPCLPointCloud2 (*cloud_filtered, *last_perception_);

	tf::StampedTransform sensorToBaseTf;
	try {
		m_tfListener.lookupTransform(m_baseFrameId, cloud_in->header.frame_id, cloud_in->header.stamp, sensorToBaseTf);
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
WmGlobalNavigation::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_in)
{

	pose = *pose_in;
	ROS_DEBUG("Pose received (%f, %f)",  pose.pose.pose.position.x, pose.pose.pose.position.y);
}

void
WmGlobalNavigation::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_in)
{
	*goal = *goal_in;

	has_goal = true;
}


void
WmGlobalNavigation::mapCallback(const sensor_msgs::PointCloud2::ConstPtr& map_in)
{

	ros::WallTime startTime = ros::WallTime::now();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_in_xyz (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());

	pcl_conversions::toPCL(*map_in, *cloud);
	pcl::fromPCLPointCloud2 (*cloud, *map_in_xyz);

	map->setInputCloud (map_in_xyz);
	map->addPointsFromInputCloud ();


	ROS_DEBUG("Map Pointcloud %zu",  map_in_xyz->size());

	if(!map_in_xyz->empty())
	{
		ROS_INFO("Map Received; Unsubscribing from mapserver");
		m_mapSub->unsubscribe();
	}
	pcl::PointCloud<pcl::PointXYZRGB>::iterator it;

	map_max_x = map_max_y =FLT_MIN;
	map_min_x = map_min_y =FLT_MIN;


	for(it=map_in_xyz->begin(); it!=map_in_xyz->end(); ++it)
	{
		if(it->x < map_min_x) map_min_x = it->x;
		if(it->y < map_min_y) map_min_y = it->y;
		if(it->x > map_max_x) map_max_x = it->x;
		if(it->y > map_max_y) map_max_y = it->y;
	}

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Map reception [ %f -> %f, %f -> %f ]done (%zu pts total, %f sec)",
			map_min_x, map_max_x, map_min_y, map_max_y, map_in_xyz->size(), total_elapsed);

	static_costmap.resizeMap((map_max_x-map_min_x)*10, (map_max_y-map_min_y)*10, 0.1, map_min_x, map_min_y);
	dynamic_costmap.resizeMap((map_max_x-map_min_x)*10, (map_max_y-map_min_y)*10, 0.1, map_min_x, map_min_y);
	goal_gradient.resizeMap((map_max_x-map_min_x)*10, (map_max_y-map_min_y)*10, 0.1, map_min_x, map_min_y);

	static_costmap.setDefaultValue(255);
	static_costmap.resetMap(0,0,static_costmap.getSizeInCellsX(), static_costmap.getSizeInCellsY());

	goal_gradient.setDefaultValue(255);
	goal_gradient.resetMap(0,0,goal_gradient.getSizeInCellsX(), goal_gradient.getSizeInCellsY());

	dynamic_costmap.setDefaultValue(0);
	dynamic_costmap.resetMap(0,0,dynamic_costmap.getSizeInCellsX(), dynamic_costmap.getSizeInCellsY());
	updateStaticCostmap();

	//costmap_pub.publishCostmap();
	//goal_gradient_pub.publishCostmap();

	return;
}

void
WmGlobalNavigation::updateDynamicCostmap()
{
	double total_elapsed = (ros::WallTime::now() - last_dynamic_map_update).toSec();

	for(int i=0; i<dynamic_costmap.getSizeInCellsX(); i++)
		for(int j=0; j<dynamic_costmap.getSizeInCellsY(); j++)
		{
			double new_cost;
			new_cost = dynamic_costmap.getCost(i, j) - (dynamic_cost_dec*total_elapsed);
			if(new_cost < 0) new_cost=0;

			if(i==dynamic_costmap.getSizeInCellsX()-1 && j==dynamic_costmap.getSizeInCellsY()-1)
				ROS_DEBUG("(%d, %lf)", dynamic_costmap.getCost(i, j), new_cost);

			dynamic_costmap.setCost(i, j, new_cost); //10 unids/sec
	}


	tf::StampedTransform robot2MapTf;
	robot2MapTf.frame_id_=m_baseFrameId;
	robot2MapTf.child_frame_id_=m_worldFrameId;
	robot2MapTf.stamp_=ros::Time::now();
	robot2MapTf.setOrigin(tf::Vector3(pose.pose.pose.position.x, pose.pose.pose.position.y,
			pose.pose.pose.position.z));
	robot2MapTf.setRotation(tf::Quaternion(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
			pose.pose.pose.orientation.z, pose.pose.pose.orientation.w ));

	Eigen::Matrix4f robot2Map;
	pcl_ros::transformAsMatrix(robot2MapTf, robot2Map);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_perception_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*last_perception_, *last_perception_temp, robot2Map);


	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr perception_octree(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(128.0f));

	perception_octree->setInputCloud (last_perception_temp);
	perception_octree->addPointsFromInputCloud ();

	for(int i=0; i<dynamic_costmap.getSizeInCellsX(); i++)
		for(int j=0; j<dynamic_costmap.getSizeInCellsY(); j++)
		{
			pcl::PointXYZRGB gridPoint;

			ij2xy(dynamic_costmap, i,j, gridPoint.x, gridPoint.y);

			gridPoint.z = 0.5;

			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			int npoints;

			npoints = perception_octree->radiusSearch(gridPoint, 0.3, pointIdxRadiusSearch, pointRadiusSquaredDistance);

			if(npoints>0)
			{


				double new_cost;
				new_cost =  dynamic_costmap.getCost(i, j) + (dynamic_cost_inc*total_elapsed);
				if(new_cost>255) new_cost=255;

				//ROS_DEBUG("(%d, %d)\t%d -> %lf", i, j, dynamic_costmap.getCost(i, j), new_cost);

				dynamic_costmap.setCost(i, j, new_cost);
			}

		}
	/*
	ROS_DEBUG("Last_perception is %ld points", last_perception_->size());

	pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
	for(it=last_perception_temp->begin(); it!=last_perception_temp->end(); ++it)
	{
		int i, j;
		xy2ij(dynamic_costmap, it->x, it->y, i, j);
		double new_cost;

		new_cost =  dynamic_costmap.getCost(i, j) + (10.0*total_elapsed);

		if(new_cost>255) new_cost=255;

		dynamic_costmap.setCost(i, j, new_cost);

		//ROS_DEBUG("(%d, %d) -> %lf", i, j, new_cost);
	}*/

	last_dynamic_map_update = ros::WallTime::now();
}

void
WmGlobalNavigation::updateStaticCostmap()
{

	for(int i=0; i<static_costmap.getSizeInCellsX(); i++)
		for(int j=0; j<static_costmap.getSizeInCellsY(); j++)
		{
			pcl::PointXYZRGB gridPoint;

			ij2xy(static_costmap, i,j, gridPoint.x, gridPoint.y);

			gridPoint.z = 0.5;

			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			int npoints;

			npoints = map->radiusSearch(gridPoint, 0.3, pointIdxRadiusSearch, pointRadiusSquaredDistance);

			if(npoints==0)
				static_costmap.setCost(i, j, 0);
			else
			{
				std::sort(pointRadiusSquaredDistance.begin(), pointRadiusSquaredDistance.end());
				static_costmap.setCost(i, j, (1.0-(pointRadiusSquaredDistance[0]/0.3)*255));
			}

		}
}

void
WmGlobalNavigation::updateGradient(int i, int j, int cost)
{
	if(i<0) return;
	if(i>(goal_gradient.getSizeInCellsX()-1)) return;
	if(j<0) return;
	if(j>(goal_gradient.getSizeInCellsY()-1)) return;

	if(static_costmap.getCost(i,j)>0) return;
	if(dynamic_costmap.getCost(i,j)>0) return;

	if(goal_gradient.getCost(i,j)<=cost)
		return;

	goal_gradient.setCost(i,j, cost);

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

	goal_gradient.resetMap(0,0,goal_gradient.getSizeInCellsX(), goal_gradient.getSizeInCellsY());

	int i, j;

	xy2ij(goal_gradient, goal->pose.position.x, goal->pose.position.y, i, j);

	updateGradient(i, j, 0);

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("UpdatePath done (%f sec)", total_elapsed);

}

void
WmGlobalNavigation::step()
{

	ros::WallTime startTime = ros::WallTime::now();
	if((map->getInputCloud()==NULL)||(map->getInputCloud()->empty()))
		return;

	updateDynamicCostmap();

	if(has_goal)
	{
		updatePath();

		int robot_i, robot_j;
		xy2ij(goal_gradient, pose.pose.pose.position.x, pose.pose.pose.position.y, robot_i, robot_j);

		int min;
		min = goal_gradient.getCost(robot_i, robot_j);


		std::vector<int> menores_i;
		std::vector<int> menores_j;
		for(int i=-3; i<=3; i++)
			for(int j=-4; j<=4; j++)
			{
				if(goal_gradient.getCost(robot_i+i, robot_j+j)<min)
				{
					menores_i.clear();
					menores_j.clear();
					min = goal_gradient.getCost(robot_i+i, robot_j+j);
					menores_i.push_back(robot_i+i);
					menores_j.push_back(robot_j+j);
				}
				if(goal_gradient.getCost(robot_i+i, robot_j+j)==min)
				{
					menores_i.push_back(robot_i+i);
					menores_j.push_back(robot_j+j);
				}
			}

		double g_x, g_y;
		g_x = g_y = 0.0;


		double dist2goal;
		double robot_x, robot_y, goal_x, goal_y;
		robot_x = pose.pose.pose.position.x;
		robot_y = pose.pose.pose.position.y;
		goal_x = goal->pose.position.x;
		goal_y = goal->pose.position.y;

		dist2goal = sqrt((robot_x-goal_x)*(robot_x-goal_x)+(robot_y-goal_y)*(robot_y-goal_y));

		ROS_DEBUG("DISTANCE2GOAL = %lf", dist2goal);
		if(dist2goal<0.20)
		{

			//has_goal = false;

			goal_vector->twist.linear.x = 0.0;

			double r, p, angle2goal;
			tf::Quaternion q1(goal->pose.orientation.x, goal->pose.orientation.y, goal->pose.orientation.z, goal->pose.orientation.w);
			tf::Matrix3x3(q1).getRPY(r, p, angle2goal);

			double robot_angle;
			tf::Quaternion q2(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
			tf::Matrix3x3(q2).getRPY(r, p, robot_angle);

			goal_vector->twist.angular.z = normalizePi(angle2goal-robot_angle);

			ROS_DEBUG("(%lf - %lf -> %lf", angle2goal, robot_angle,  normalizePi(angle2goal-robot_angle));

		}else
		{
			for(int i=0; i<menores_i.size();++i)
			{
				double t_x, t_y;
				ij2xy(static_costmap, menores_i[i], menores_j[i], t_x, t_y);

				g_x = g_x + (t_x-g_x)/(i+1);
				g_y = g_y + (t_y-g_y)/(i+1);
			}



			goal_vector->twist.linear.x = dist2goal/5.0;


			double angle2goal;
			angle2goal = atan2(g_y - pose.pose.pose.position.y, g_x - pose.pose.pose.position.x);

			double r, p, robot_angle;
			tf::Quaternion q(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
			tf::Matrix3x3(q).getRPY(r, p, robot_angle);

			goal_vector->twist.angular.z = normalizePi(angle2goal-robot_angle);

			ROS_DEBUG("GLOBAL VECTOR = (%lf, %lf)", goal_vector->twist.linear.x, goal_vector->twist.angular.z);
		}


		double total_elapsed = (ros::WallTime::now() - startTime).toSec();
		ROS_DEBUG("GlobalNavigation done (%f sec)", total_elapsed);
	}
	publish_all();


}

void
WmGlobalNavigation::publish_all()
{
	publish_static_map();
	publish_dynamic_map();
	publish_gradient();
	if(has_goal) publish_goal();
}

void
WmGlobalNavigation::publish_static_map()
{
	static_costmap_pub.publishCostmap();
}
void
WmGlobalNavigation::publish_dynamic_map()
{
	dynamic_costmap_pub.publishCostmap();
}


void
WmGlobalNavigation::publish_gradient()
{
	goal_gradient_pub.publishCostmap();
}

void
WmGlobalNavigation::publish_goal()
{
	goal_vector->header.frame_id = m_baseFrameId;
	goal_vector->header.stamp = ros::Time::now();

	/*

	tf::Transform G2L;
	G2L.setOrigin( tf::Vector3(pose.pose.pose.position.x, pose.pose.pose.position.y,
			pose.pose.pose.position.z) );
	G2L.setRotation( tf::Quaternion(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
			pose.pose.pose.orientation.z, pose.pose.pose.orientation.w) );

	tf::Transform G;
	G.setOrigin( tf::Vector3(goal_vector->pose.position.x,
			goal_vector->pose.position.y,
			goal_vector->pose.position.z));
	G.setRotation( tf::Quaternion(goal_vector->pose.orientation.x, goal_vector->pose.orientation.y,
			goal_vector->pose.orientation.z, goal_vector->pose.orientation.w) );

	tf::Transform R;

	R = G2L.inverse()*G;

	goal_vector->pose.position.x = R.getOrigin().getX();
	goal_vector->pose.position.y = R.getOrigin().getY();
	goal_vector->pose.position.z = R.getOrigin().getZ();


	goal_vector->pose.orientation.x = R.getRotation().getX();
	goal_vector->pose.orientation.y = R.getRotation().getY();
	goal_vector->pose.orientation.z = R.getRotation().getZ();
	goal_vector->pose.orientation.w = R.getRotation().getW();
*/

	goal_vector_pub.publish(goal_vector);

}

}

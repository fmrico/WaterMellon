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
  m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
  map(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(128.0f)),
  map_max_x(1.0), map_min_x(0.0), map_max_y(1.0), map_min_y(0.0),
  m_tfListener(),
  costmap(),
  goal_gradient(),
  costmap_pub(&m_nh, &costmap, "/map", "/wm_navigation_costmap", true),
  goal_gradient_pub(&m_nh, &goal_gradient, "/map", "/wm_navigation_goal_gradient", true),
  goal_vector(new geometry_msgs::PoseStamped),
  goal(new geometry_msgs::PoseStamped),
  has_goal(false)
{
	ros::NodeHandle private_nh(private_nh_);
	private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
	private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);


	m_mapSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "map", 5);
	m_tfMapSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_mapSub, m_tfListener, m_worldFrameId, 5);
	m_tfMapSub->registerCallback(boost::bind(&WmGlobalNavigation::mapCallback, this, _1));

	goal_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &WmGlobalNavigation::goalCallback, this);
	pose_sub = m_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/wm_pose_cov", 1, &WmGlobalNavigation::poseCallback, this);
	goal_vector_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/goal_vector", 1000);

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
		ros::console::notifyLoggerLevelsChanged();



}

WmGlobalNavigation::~WmGlobalNavigation() {

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

	for(int i=0; i<costmap.getSizeInCellsX(); i++)
		for(int j=0; j<costmap.getSizeInCellsY(); j++)

			goal_gradient.resetMap(0,0,goal_gradient.getSizeInCellsX(), goal_gradient.getSizeInCellsY());

	int i, j;

	xy2ij(goal->pose.position.x, goal->pose.position.y, i, j);

	updateGradient(i, j, 0);

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

	costmap.resizeMap((map_max_x-map_min_x)*10, (map_max_y-map_min_y)*10, 0.1, map_min_x, map_min_y);
	goal_gradient.resizeMap((map_max_x-map_min_x)*10, (map_max_y-map_min_y)*10, 0.1, map_min_x, map_min_y);

	goal_gradient.setDefaultValue(255);
	goal_gradient.resetMap(0,0,goal_gradient.getSizeInCellsX(), goal_gradient.getSizeInCellsY());

	updateCostmap();

	//costmap_pub.publishCostmap();
	//goal_gradient_pub.publishCostmap();

	return;
}

void
WmGlobalNavigation::updateCostmap()
{

	for(int i=0; i<costmap.getSizeInCellsX(); i++)
		for(int j=0; j<costmap.getSizeInCellsY(); j++)
		{
			pcl::PointXYZRGB gridPoint;

			ij2xy(i,j, gridPoint.x, gridPoint.y);

			gridPoint.z = 0.5;

			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			int npoints;

			npoints = map->radiusSearch(gridPoint, 0.3, pointIdxRadiusSearch, pointRadiusSquaredDistance);

			if(npoints==0)
				costmap.setCost(i, j, 0);
			else
			{
				std::sort(pointRadiusSquaredDistance.begin(), pointRadiusSquaredDistance.end());
				costmap.setCost(i, j, (1.0-(pointRadiusSquaredDistance[0]/0.3)*255));
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

	if(costmap.getCost(i,j)>0) return;

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
WmGlobalNavigation::step()
{

	ros::WallTime startTime = ros::WallTime::now();
	if((map->getInputCloud()==NULL)||(map->getInputCloud()->empty()))
		return;

	if(has_goal)
	{
		int robot_i, robot_j;
		xy2ij(pose.pose.pose.position.x, pose.pose.pose.position.y, robot_i, robot_j);

		int min;
		min = goal_gradient.getCost(robot_i, robot_j);


		std::vector<int> menores_i;
		std::vector<int> menores_j;
		for(int i=-3; i<=3; i++)
			for(int j=-5; j<=5; j++)
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
		if(min == goal_gradient.getCost(robot_i, robot_j))
		{
			ROS_INFO("GLOBAL GOAL REACHED)");
			has_goal = false;
		}else
		{
			for(int i=0; i<menores_i.size();++i)
			{
				double t_x, t_y;
				ij2xy(menores_i[i], menores_j[i], t_x, t_y);

				g_x = g_x + (t_x-g_x)/(i+1);
				g_y = g_y + (t_y-g_y)/(i+1);
			}
		}

		double yaw;
		yaw = atan2(g_y - pose.pose.pose.position.y, g_x - pose.pose.pose.position.x);

		goal_vector->pose.position = pose.pose.pose.position;

		tf::Quaternion q;
		q.setEuler(0.0, 0.0, yaw);
		goal_vector->pose.orientation.x = q.x();
		goal_vector->pose.orientation.y = q.y();
		goal_vector->pose.orientation.z = q.z();
		goal_vector->pose.orientation.w = q.w();

		double total_elapsed = (ros::WallTime::now() - startTime).toSec();
		ROS_DEBUG("GlobalNavigation done (%f sec)", total_elapsed);
	}
	publish_all();


}

void
WmGlobalNavigation::publish_all()
{
	publish_map();
	publish_gradient();
	if(has_goal) publish_goal();
}

void
WmGlobalNavigation::publish_map()
{
	costmap_pub.publishCostmap();
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


	goal_vector_pub.publish(goal_vector);

}

}

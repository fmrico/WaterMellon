/*
 * mapper.cpp
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */

#include "wm_navigation/WmLocalNavigation.h"

namespace wm_navigation{


WmLocalNavigation::WmLocalNavigation(ros::NodeHandle private_nh_)
: m_nh(),
  m_pointcloudMinZ(0.2),
  m_pointcloudMaxZ(0.5),
  robot_radious(0.3),
  max_w(0.15),
  max_v(0.2),
  collision_area(1.0),
  m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
  m_tfListener(),
  //last_perception(new pcl::PointCloud<pcl::PointXYZRGB>),
  global_vector(new geometry_msgs::TwistStamped),
  resultant_vector(new geometry_msgs::TwistStamped),
  //repulsive_vector(new geometry_msgs::PoseStamped),
  m_res(0.2)
{
	ros::NodeHandle private_nh(private_nh_);
	private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
	private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
	private_nh.param("robot_radious", robot_radious, robot_radious);
	private_nh.param("collision_area", collision_area, collision_area);


	/*m_perceptSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
	m_tfPerceptSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_perceptSub, m_tfListener, m_baseFrameId, 5);
	m_tfPerceptSub->registerCallback(boost::bind(&WmLocalNavigation::perceptionCallback, this, _1));
*/
	goal_sub = m_nh.subscribe<geometry_msgs::TwistStamped>("/goal_vector", 1000, &WmLocalNavigation::gvectorCallback, this);

	//repulsive_vector_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/repulsive_vector", 1000);
	resultant_vector_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/resultant_vector", 1000);

	vel_pub = m_nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
//	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
//		ros::console::notifyLoggerLevelsChanged();


}

WmLocalNavigation::~WmLocalNavigation() {

}

/*
void
WmLocalNavigation::perceptionCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
	ros::WallTime startTime = ros::WallTime::now();

	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

	pcl_conversions::toPCL(*cloud_in, *cloud);

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (m_res, m_res, m_res);
	sor.filter (*cloud_filtered);

	pcl::fromPCLPointCloud2 (*cloud_filtered, *last_perception);

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
	pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

	// directly transform to map frame:
	pcl::transformPointCloud(*last_perception, *last_perception, sensorToBase);

	// just filter height range:
	pass.setInputCloud(last_perception->makeShared());
	pass.filter(*last_perception);

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();

	return;
}

*/
void
WmLocalNavigation::gvectorCallback(const geometry_msgs::TwistStamped::ConstPtr& gvector_in)
{

	ROS_DEBUG("Global Vector received");
	*global_vector = *gvector_in;

}


void
WmLocalNavigation::step()
{

	ros::WallTime startTime = ros::WallTime::now();
	double total_elapsed = (ros::WallTime::now() - startTime).toSec();

	/*
	pcl::PointCloud<pcl::PointXYZRGB>::iterator it;

	float r_x, r_y;
	int count =0;
	r_x = 0.0;
	r_y = 0.0;

	for(it=last_perception->begin();it!=last_perception->end(); ++it)
	{

		float dist, rvector, ang;

		dist= sqrt(it->x*it->x+it->y*it->y);
		ang = atan2(it->y, it->x)+ M_PI;

		if(ang>M_PI) ang = ang - 2.0*M_PI;

		if(dist<collision_area)
		{

			rvector = 1.0 - ((dist-robot_radious)/(collision_area-robot_radious));
			//rvector = pow((dist - robot_radious) - collision_area, 2.0)/ (robot_radious*robot_radious);
			//std::cerr<<"Dist = "<<dist<<"\tfr = "<<rvector<<std::endl;

			count++;
			r_x = r_x + (rvector*cos(ang)-r_x)/(count);
			r_y = r_y + (rvector*sin(ang)-r_y)/(count);
		}

	}

	std::cerr<<"REPULSIVE = ("<<r_x<<", "<<r_y<<")"<<std::endl;

	r_x = r_x * 2.0;
	r_y = r_y * 2.0;

	 /*
	double ax, ay;
	double roll, pitch, yaw;
	tf::Quaternion q(global_vector->pose.orientation.x, global_vector->pose.orientation.y,
			global_vector->pose.orientation.z, global_vector->pose.orientation.w);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	global_vector->twist.angular.z

	ax = cos(global_vector->twist.angular.z);
	ay = sin(global_vector->twist.angular.z);

	std::cerr<<"ATRACTIVE = ("<<ax<<", "<<ay<<")"<<std::endl;

	double resx, resy;

	resx = ax;//(ax + r_x)/2.0;
	resy = ay;//(ay + r_y)/2.0;

	std::cerr<<"RES = ("<<resx<<", "<<resy<<")"<<std::endl;

	 */

	/*tf::Quaternion qrep;
	qrep.setEuler(0.0, 0.0, atan2(r_y, r_x));
	repulsive_vector->pose.position.x = 0.0f;
	repulsive_vector->pose.position.y = 0.0f;
	repulsive_vector->pose.position.z = 0.0f;
	repulsive_vector->pose.orientation.x = qrep.getX();
	repulsive_vector->pose.orientation.y = qrep.getY();
	repulsive_vector->pose.orientation.z = qrep.getZ();
	repulsive_vector->pose.orientation.w = qrep.getW();
*/
	/*tf::Quaternion qres;
	qres.setEuler(0.0, 0.0, atan2(resy, resx));
	resultant_vector->pose.position.x = 0.0f;
	resultant_vector->pose.position.y = 0.0f;
	resultant_vector->pose.position.z = 0.0f;
	resultant_vector->pose.orientation.x = qres.getX();
	resultant_vector->pose.orientation.y = qres.getY();
	resultant_vector->pose.orientation.z = qres.getZ();
	resultant_vector->pose.orientation.w = qres.getW();


	if(!std::isnan(resy))
	{
		geometry_msgs::Twist vel;



		if(fabs(atan2(resy, resx))>0.5)
		{
			vel.linear.x = 0.0;
			vel.angular.z = atan2(resy, resx);
		}else
		{
			vel.linear.x = resx;
			vel.angular.z = atan2(resy, resx);
		}


		if(vel.angular.z>max_w)
			vel.angular.z = max_w;
		if(vel.angular.z<-max_w)
			vel.angular.z = -max_w;
		if(vel.linear.x > max_v)
			vel.linear.x = max_v;
		if(vel.linear.x < -max_v)
			vel.linear.x = -max_v;

		vel_pub.publish(vel);
	}

	 */

	geometry_msgs::Twist vel;
	//vel = global_vector->twist;

	if(fabs(global_vector->twist.angular.z) > 0.5)
	{
		vel.linear.x = 0.0;
		vel.angular.z = global_vector->twist.angular.z;
	}else
	{
		vel.linear.x = global_vector->twist.linear.x;
		vel.angular.z = global_vector->twist.angular.z;
	}

	if(vel.angular.z>max_w)
		vel.angular.z = max_w;
	if(vel.angular.z<-max_w)
		vel.angular.z = -max_w;
	if(vel.linear.x > max_v)
		vel.linear.x = max_v;
	if(vel.linear.x < -max_v)
		vel.linear.x = -max_v;

	vel_pub.publish(vel);

	//vel.linear.x = global_vector->twist.linear.x;
	//vel. = global_vector->twist.linear.x;

	ROS_DEBUG("LOCAL VEL = (%lf, %lf)", vel.linear.x, vel.angular.z);

	ROS_DEBUG("LocalNavigation done (%f sec)", total_elapsed);

	publish_all();


}

void
WmLocalNavigation::publish_all()
{

	publish_resultant_vector();
	//publish_repulsive_vector();
}

/*
void
WmLocalNavigation::publish_repulsive_vector()
{
	repulsive_vector->header.frame_id = m_baseFrameId;
	repulsive_vector->header.stamp = ros::Time::now();

	repulsive_vector_pub.publish(repulsive_vector);

}*/
void
WmLocalNavigation::publish_resultant_vector()
{
	resultant_vector->header.frame_id = m_baseFrameId;
	resultant_vector->header.stamp = ros::Time::now();

	resultant_vector_pub.publish(resultant_vector);

}

}

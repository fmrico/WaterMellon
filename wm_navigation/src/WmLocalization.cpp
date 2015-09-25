/*
 * mapper.cpp
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */

#include "wm_navigation/WmLocalization.h"

namespace wm_localization{


WmLocalization::WmLocalization(ros::NodeHandle private_nh_)
: m_nh(),
  m_pointcloudMinZ(0.2),
  m_pointcloudMaxZ(0.5),
  m_res(0.05),
  m_numparticles_min(30),
  m_numparticles_max(300),
  m_correctpoints(20),
  m_odomerror(0.15),
  m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
  last_perception(new pcl::PointCloud<pcl::PointXYZRGB>),
  map(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(128.0f)),
  map_max_x(1.0), map_min_x(0.0), map_max_y(1.0), map_min_y(0.0),
  doResetParticles(true),
  particles(m_numparticles_max)
{
	ros::NodeHandle private_nh(private_nh_);
	private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
	private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
	private_nh.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
	private_nh.param("pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
	private_nh.param("resolution", m_res, m_res);
	private_nh.param("numparticles_min", m_numparticles_min, m_numparticles_min);
	private_nh.param("numparticles_max", m_numparticles_max, m_numparticles_max);
	private_nh.param("correctpoints", m_correctpoints, m_correctpoints);
	private_nh.param("odomerror", m_odomerror, m_odomerror);


	m_perceptSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
	m_tfPerceptSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_perceptSub, m_tfListener, m_baseFrameId, 5);
	m_tfPerceptSub->registerCallback(boost::bind(&WmLocalization::perceptionCallback, this, _1));

	m_mapSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "map", 5);
	m_tfMapSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_mapSub, m_tfListener, m_worldFrameId, 5);
	m_tfMapSub->registerCallback(boost::bind(&WmLocalization::mapCallback, this, _1));

	perception_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/wm_perception", 1, true);
	particles_pub = m_nh.advertise<geometry_msgs::PoseArray>("/wm_particles", 1, true);
	pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/wm_pose", 1, true);
	posecov_pub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/wm_pose_cov", 1, true);

	testpoints_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/wm_testpoints", 1, true);


	srand(time(0));

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
		ros::console::notifyLoggerLevelsChanged();


}

WmLocalization::~WmLocalization() {

}

void
WmLocalization::perceptionCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
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
	//ROS_DEBUG("Pointcloud perception done (%zu pts total, %f sec)", last_perception->size(), total_elapsed);

	return;
}


void
WmLocalization::mapCallback(const sensor_msgs::PointCloud2::ConstPtr& map_in)
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

	return;
}

bool
WmLocalization::validParticlePosition(float x, float y)
{

	if((map->getInputCloud()==NULL)||(map->getInputCloud()->empty()))
		return true;

	bool infloor, inwall;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	pcl::PointXYZRGB searchPointf, searchPointw;

	searchPointf.x = x;
	searchPointf.y = y;
	searchPointf.z = 0.0;
	searchPointw.x = x;
	searchPointw.y = y;
	searchPointw.z = m_res*4.0f;

	infloor = map->radiusSearch(searchPointf, m_res*2.0f, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0;
	inwall = map->radiusSearch(searchPointw, m_res*2.0f, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0;

	if(infloor && !inwall)
		return true;
	else
	{
		//std::cerr<<"*";
		return false;
	}
}

void
WmLocalization::resetParticleUniform(Particle& p)
{
	float x, y, t;
	p.p = 1.0 / ((float) particles.size());

	do {
		x = ((float) rand() / (float) RAND_MAX) * (map_max_x-map_min_x) + map_min_x;
		y = ((float) rand() / (float) RAND_MAX) * (map_max_y-map_min_y) + map_min_y;
	} while (!validParticlePosition(x,y));

	t = normalizePi(((float) rand() / (float) RAND_MAX) * 2.0 * M_PI);

	p.coord.position.x = x;
	p.coord.position.y = y;
	p.coord.position.z = 0.0;

	tf::Quaternion q;
	q.setEuler(0.0, 0.0, t);

	p.coord.orientation.x = q.x();
	p.coord.orientation.y = q.y();
	p.coord.orientation.z = q.z();
	p.coord.orientation.w = q.w();
}

void
WmLocalization::resetParticleNear(Particle& p2reset, const Particle& pref)
{
	float x, y;


	//do {
		x = pref.coord.position.x + randn(0.0, 0.05);//normalX(generator);
		y = pref.coord.position.y + randn(0.0, 0.05);//normalY(generator);
	//} while (!validParticlePosition(x,y));

	tf::Quaternion q(pref.coord.orientation.x, pref.coord.orientation.y,
			pref.coord.orientation.z, pref.coord.orientation.w);

	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	float newt = normalizePi(yaw + randn(0.0, 0.2));

	setParticle(p2reset, x, y, newt);

	p2reset.p = 1.0/(float)particles.size();
}

void
WmLocalization::resetParticles()
{
	ros::WallTime startTime = ros::WallTime::now();

	for (int i = 0; i < particles.size(); i++)
		resetParticleUniform(particles[i]);

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	//ROS_DEBUG("Particles reset took %f sec", total_elapsed);
}

void
WmLocalization::publishAll()
{
	//publishPerception();
	publishParticles();
	publishPose();
	publishPoseWithCov();
}

void
WmLocalization::publishPoseWithCov(const ros::Time& rostime)
{
	bool publishPose = posecov_pub.getNumSubscribers() > 0;

	if(publishPose)
	{
		geometry_msgs::PoseWithCovarianceStamped pose2send;

		pose2send.header.frame_id = "/map";
		pose2send.header.stamp = ros::Time::now();
		pose2send.pose = pose;

		posecov_pub.publish(pose2send);
	}
}

void
WmLocalization::publishPose(const ros::Time& rostime)
{
	bool publishPose = pose_pub.getNumSubscribers() > 0;

	if(publishPose)
	{
		geometry_msgs::PoseStamped pose2send;

		pose2send.header.frame_id = "/map";
		pose2send.header.stamp = ros::Time::now();
		pose2send.pose = pose.pose;

		pose_pub.publish(pose2send);
	}
}

void
WmLocalization::publishParticles(const ros::Time& rostime)
{
	bool publishParticles = particles_pub.getNumSubscribers() > 0;

	if (publishParticles){
		geometry_msgs::PoseArray paray;

		paray.header.frame_id = "/map";
		paray.header.stamp = ros::Time::now();

		for (int i = 0; i < particles.size(); i++)
			paray.poses.push_back(particles[i].coord);

		particles_pub.publish(paray);
	}
}

void
WmLocalization::publishPerception(const ros::Time& rostime)
{
	size_t perceptionSize = last_perception->size();

	if (perceptionSize <= 1){
		ROS_WARN("Nothing to publish, perception is empty");
		return;
	}

	bool publishPointCloud = perception_pub.getNumSubscribers() > 0;

	// finish pointcloud:
	if (publishPointCloud){
		sensor_msgs::PointCloud2 cloud;
		pcl::toROSMsg (*last_perception, cloud);
		cloud.header.frame_id = m_baseFrameId;
		cloud.header.stamp = rostime;
		perception_pub.publish(cloud);
	}
}


void
WmLocalization::step()
{

	ros::WallTime startTime = ros::WallTime::now();
	if((map->getInputCloud()==NULL)||(map->getInputCloud()->empty()))
		return;


	if(doResetParticles)
	{
		resetParticles();
		doResetParticles = false;
	}


	predict();
	correct();

	reseed();

	normalize();

	updatePos();

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Localization done (%f sec)", total_elapsed);

	publishAll();
}

tf::Transform
WmLocalization::genNoise(tf::Transform &base)
{

	tf::Transform ret;

	float nx, ny, nz, nt;

	nx = randn(0.0, 0.05);//(*distribution)(generator);
	ny = randn(0.0, 0.05);//(*distribution)(generator);
	nz = 0.0f;
	nt = randn(0.0, 0.05);//(*distribution)(generator);


	ret.setOrigin(tf::Vector3(base.getOrigin().getX()*nx,
			base.getOrigin().getY()*ny,
			base.getOrigin().getZ()*nz));


	tf::Quaternion q(base.getRotation().getX(),
			base.getRotation().getY(),
			base.getRotation().getZ(),
			base.getRotation().getW());

	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	yaw = yaw * nt;

	q.setEuler(roll, pitch, yaw);

	ret.setRotation(q);

	return ret;

}

void
WmLocalization::predict()
{
	tf::Transform desp;
		tf::StampedTransform odom2bf;

		try {
			m_tfListener.lookupTransform("/odom", "base_footprint",
					ros::Time(0), odom2bf);

		} catch (tf::TransformException & ex) {
			ROS_WARN("%s", ex.what());
		}

		desp = last2odom * odom2bf;

		last2odom.setOrigin(odom2bf.inverse().getOrigin());
		last2odom.setRotation(odom2bf.inverse().getRotation());


		for (int i = 0; i < particles.size(); i++) {
			tf::Transform part, parttf;

			part.setOrigin(
					tf::Vector3(particles[i].coord.position.x,
							particles[i].coord.position.y,
							particles[i].coord.position.z));
			part.setRotation(
					tf::Quaternion(particles[i].coord.orientation.x,
							particles[i].coord.orientation.y,
							particles[i].coord.orientation.z,
							particles[i].coord.orientation.w));

			tf::Transform noise = genNoise(desp);

			parttf = part * desp * noise;

			particles[i].coord.position.x = parttf.getOrigin().getX();
			particles[i].coord.position.y = parttf.getOrigin().getY();
			particles[i].coord.position.z = parttf.getOrigin().getZ();
			particles[i].coord.orientation.x = parttf.getRotation().getX();
			particles[i].coord.orientation.y = parttf.getRotation().getY();
			particles[i].coord.orientation.z = parttf.getRotation().getZ();
			particles[i].coord.orientation.w = parttf.getRotation().getW();

		}
}


float
WmLocalization::doTestPcl(pcl::PointXYZRGB &searchPoint)
{
	float ret=0.0f;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	float KO=0.0;
	float KH=0.6;
	float KS=0.2;
	float KV=0.2;

	int npoints;

	npoints = map->radiusSearch(searchPoint, m_res, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	int inc=2;
	for(int i=0; i<npoints; i=i+inc)
	{
		pcl::PointXYZHSV hsvA, hsvB;
		float diffH, diffS, diffV;

		pcl::PointXYZRGB point = map->getInputCloud()->points[pointIdxRadiusSearch[i]];

		PointXYZRGBtoXYZHSV(searchPoint, hsvA);
		PointXYZRGBtoXYZHSV(point, hsvB);

		diffH = fabs(normalizePi(toRadians(hsvA.h - 180.0) - toRadians(hsvB.h - 180.0))/M_PI);

		diffS = fabs(hsvA.s - hsvB.s);
		diffV = fabs(hsvA.v - hsvB.v);

		float vd = (m_res-sqrt(pointRadiusSquaredDistance[0]))/m_res;

		ret += (KO * vd + KH * (1.0 - diffH) + KS * (1.0 - diffS)+ KV * (1.0 - diffV))*((m_res-pointRadiusSquaredDistance[i])/m_res);

	}

	if(npoints>0)
		return ret/(npoints/inc);
	else
		return ret;
}


void
WmLocalization::updateParticle(Particle& p, const std::vector<pcl::PointXYZRGB>& testpoints)
{
	tf::Transform W2H;

	W2H.setOrigin(tf::Vector3(p.coord.position.x, p.coord.position.y, p.coord.position.z));

	W2H.setRotation( tf::Quaternion(p.coord.orientation.x, p.coord.orientation.y,
			p.coord.orientation.z, p.coord.orientation.w));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr testp(new pcl::PointCloud<pcl::PointXYZRGB>);

	float temp = 0.0;;
	//p.p = 0.0f;

	for (int i = 0; i < m_correctpoints; i++)
	{
		pcl::PointXYZRGB testpoint = testpoints[i];

		tf::Vector3 posc(testpoint.x, testpoint.y, testpoint.z);
		tf::Vector3 posw;

		posw = W2H * posc;

		testpoint.x = posw.getX();
		testpoint.y = posw.getY();
		testpoint.z = posw.getZ();

		testp->push_back(testpoint);

		temp = temp + doTestPcl(testpoint);
	}

	p.p = (p.p + temp/m_correctpoints)/2.0f;

	if(p.p>1.0) p.p=1.0;

}

void
WmLocalization::correct()
{
	ros::WallTime startTime = ros::WallTime::now();

	if(last_perception->empty())
		return;


	//ToDo: Use keypoints instead of random ones
	std::vector<pcl::PointXYZRGB> testpoints(m_correctpoints);
	for (int i = 0; i < m_correctpoints; i++)
	{
		int idx = rand() % last_perception->size();
		testpoints[i] = last_perception->at(idx);
	}

	for(int j=0; j<particles.size(); j++)
		updateParticle(particles[j], testpoints);

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Correction done (%f sec)", total_elapsed);

}
void
WmLocalization::normalize() {

	float sum = 0.0;
	float factor;

	for (int i = 0; i < particles.size(); i++)
		sum = sum + particles[i].p;

	if(sum>0.0)
		factor = 1.0 / sum;
	else
		factor = 1.0;

	for (int i = 0; i < particles.size(); i++)
		particles[i].p = particles[i].p * factor;

}


void
WmLocalization::reseed()
{
	std::sort(particles.begin(), particles.end());
	std::reverse(particles.begin(), particles.end());

	float area_unc = sqrt(pose.covariance[0]) * sqrt(pose.covariance[1 * 6 + 1]);

	if(area_unc>3.0)
	{
		int new_size = particles.size() * 1.5;
		if(new_size>m_numparticles_max) new_size = m_numparticles_max;

		int add_p = new_size-particles.size();

		for(int i=0; i<add_p;i++)
			particles.push_back(Particle());
	}else if(area_unc<0.1)
	{
		int new_size = particles.size() / 1.5;
		if(new_size<m_numparticles_min) new_size = m_numparticles_min;
		int del_p = particles.size()-new_size;

		particles.erase(particles.end()-del_p, particles.end());
	}

	int idx_50 = particles.size()*50/100;
	int idx_75 = particles.size()*75/100;

	for(int i=idx_50; i<idx_75;i++)
		resetParticleUniform(particles[i]);

	for(int i=idx_75; i<particles.size();i++)
		resetParticleNear(particles[i], particles[0]);



}


void WmLocalization::updatePos()
{
	pose.pose = particles[0].coord;

	geometry_msgs::Pose error;

	error.position.x = error.position.y = error.position.z;
	error.orientation =  tf::createQuaternionMsgFromRollPitchYaw (0.0, 0.0, 0.0);

	float p_mean = 0.0f;

	int sgnificant_part = particles.size()*50/100;
	for(int i=0; i<sgnificant_part; i++)
	{
		error.position.x = error.position.x +
				(particles[i].coord.position.x -particles[0].coord.position.x) *
				(particles[i].coord.position.x -particles[0].coord.position.x);

		error.position.y = error.position.y +
				(particles[i].coord.position.y -particles[0].coord.position.y) *
				(particles[i].coord.position.y -particles[0].coord.position.y);

		error.position.z = error.position.z +
				(particles[i].coord.position.z -particles[0].coord.position.z) *
				(particles[i].coord.position.z -particles[0].coord.position.z);

		error.orientation.x = error.orientation.x +
				(particles[i].coord.orientation.x -particles[0].coord.orientation.x) *
				(particles[i].coord.orientation.x -particles[0].coord.orientation.x);

		error.orientation.y = error.orientation.y +
				(particles[i].coord.orientation.y -particles[0].coord.orientation.y) *
				(particles[i].coord.orientation.y -particles[0].coord.orientation.y);

		error.orientation.z = error.orientation.z +
				(particles[i].coord.orientation.z -particles[0].coord.orientation.z) *
				(particles[i].coord.orientation.z -particles[0].coord.orientation.z);

		error.orientation.w = error.orientation.w +
				(particles[i].coord.orientation.w -particles[0].coord.orientation.w) *
				(particles[i].coord.orientation.w -particles[0].coord.orientation.w);

		p_mean = p_mean + particles[i].p * particles[i].p;
	}

	error.position.x = error.position.x / sgnificant_part;
	error.position.y = error.position.y / sgnificant_part;
	error.position.z = error.position.z / sgnificant_part;
	error.orientation.x = error.orientation.x / sgnificant_part;
	error.orientation.y = error.orientation.y / sgnificant_part;
	error.orientation.z = error.orientation.z / sgnificant_part;
	error.orientation.w = error.orientation.w / sgnificant_part;

	p_mean = sqrt (p_mean) / sgnificant_part;
	//ToDo: Get the right covariance, including angles
	for (int i = 0; i < 36; i++)
			pose.covariance[i] = 0.0;

	pose.covariance[0] = error.position.x;
	pose.covariance[1 * 6 + 1] =error.position.y;
	pose.covariance[5 * 6 + 5] = error.position.z;

	ROS_DEBUG("[%ld]\tp: %f\tstdvev x:%f\ty:%f\tz:%f", particles.size(), p_mean, error.position.x, error.position.y, error.position.z);

}

void WmLocalization::setParticle(Particle &particle, float x, float y, float theta)
{

	particle.coord.position.x = x;
	particle.coord.position.y = y;
	particle.coord.position.z = 0.0;

	tf::Quaternion q;
	q.setEuler(0.0, 0.0, theta);

	particle.coord.orientation.x = q.x();
	particle.coord.orientation.y = q.y();
	particle.coord.orientation.z = q.z();
	particle.coord.orientation.w = q.w();

}

}

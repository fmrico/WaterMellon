/*
 * mapper.cpp
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */

#include "wm_navigation/WmLocalization.h"

namespace wm_navigation{


WmLocalization::WmLocalization(ros::NodeHandle private_nh_)
: nh_(),
  pointcloudMinZ_(-std::numeric_limits<double>::max()),
  pointcloudMaxZ_(std::numeric_limits<double>::max()),
  pointcloudMaxX_(std::numeric_limits<double>::max()),
  pointcloudMaxTxy_(M_PI),
  pointcloudMaxTxz_(M_PI),
  res_(0.05),
  numparticles_min_(30),
  numparticles_max_(200),
  correctpoints_(20),
  odomerror_(0.15),
  doResetParticles_(true),
  worldFrameId_("/map"), baseFrameId_("base_footprint"),
  last_perception_(new pcl::PointCloud<pcl::PointXYZRGB>),
  map_(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(128.0f)),
  map_max_x_(1.0), map_min_x_(0.0), map_max_y_(1.0), map_min_y_(0.0),
  particles_(numparticles_max_)
{

	ros::NodeHandle private_nh(private_nh_);
	private_nh.param("frame_id", worldFrameId_, worldFrameId_);
	private_nh.param("base_frame_id", baseFrameId_, baseFrameId_);
	private_nh.param("pointcloud_min_z", pointcloudMinZ_,pointcloudMinZ_);
	private_nh.param("pointcloud_max_z", pointcloudMaxZ_,pointcloudMaxZ_);
	private_nh.param("pointcloud_max_x", pointcloudMaxX_,pointcloudMaxX_);
	private_nh.param("pointcloud_max_t_xy", pointcloudMaxTxy_,pointcloudMaxTxy_);
	private_nh.param("pointcloud_max_t_xz", pointcloudMaxTxz_,pointcloudMaxTxz_);
	private_nh.param("resolution", res_, res_);
	private_nh.param("numparticles_min", numparticles_min_, numparticles_min_);
	private_nh.param("numparticles_max", numparticles_max_, numparticles_max_);
	private_nh.param("correctpoints", correctpoints_, correctpoints_);
	private_nh.param("odomerror", odomerror_, odomerror_);

	if(private_nh.hasParam("initial_pose_x") && private_nh.hasParam("initial_pose_y") && private_nh.hasParam("initial_pose_a"))
	{
		double initial_pose_a;
		geometry_msgs::Pose init_coord;

		private_nh.param("initial_pose_x", init_coord.position.x, init_coord.position.x);
		private_nh.param("initial_pose_y", init_coord.position.y, init_coord.position.y);
		private_nh.param("initial_pose_a", initial_pose_a, initial_pose_a);

		tf::Quaternion q;
		q.setEuler(0.0, 0.0, initial_pose_a);

		init_coord.orientation.x = q.x();
		init_coord.orientation.y = q.y();
		init_coord.orientation.z = q.z();
		init_coord.orientation.w = q.w();

		setInitPose(init_coord);
		doResetParticles_ = false;
	}

	perceptSub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "cloud_in", 5);
	tfPerceptSub_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*perceptSub_, tfListener_, baseFrameId_, 5);
	tfPerceptSub_->registerCallback(boost::bind(&WmLocalization::perceptionCallback, this, _1));

	mapSub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "map", 5);
	tfMapSub_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*mapSub_, tfListener_, worldFrameId_, 5);
	tfMapSub_->registerCallback(boost::bind(&WmLocalization::mapCallback, this, _1));

	pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000, &WmLocalization::setPosCallback, this);

	perception_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/wm_perception", 1, true);
	particles_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/wm_particles", 1, true);
	pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/wm_pose", 1, true);
	posecov_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/wm_pose_cov", 1, true);


	srand(time(0));

}

WmLocalization::~WmLocalization() {

}

void
WmLocalization::setPosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pos)
{
	setInitPose(pos->pose.pose);
}

void
WmLocalization::setInitPose(const geometry_msgs::Pose& init_coord)
{

	ROS_INFO("Set Initial Pos to (%lf, %lf, %lf)", init_coord.position.x,  init_coord.position.y,  init_coord.orientation.z);
	particles_.erase(particles_.begin()+numparticles_min_, particles_.end());

	for(int i=0; i<particles_.size(); i++)
	{
		particles_[i].coord_ = init_coord;
		particles_[i].p_ = 1.0;
	}
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
	sor.setLeafSize (res_, res_, res_);
	sor.filter (*cloud_filtered);


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>); // input cloud for filtering and ground-detection
	pcl::fromPCLPointCloud2 (*cloud_filtered, *pc);


	/*
	 * Filter in Robot space
	 */

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); // input cloud for filtering and ground-detection

	tf::StampedTransform sensorToRobotTf;
	Eigen::Matrix4f sensorToRobot;
	try {
		tfListener_.lookupTransform(baseFrameId_, cloud_in->header.frame_id, cloud_in->header.stamp, sensorToRobotTf);
	} catch(tf::TransformException& ex){
		ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}

	pcl_ros::transformAsMatrix(sensorToRobotTf, sensorToRobot);
	pcl::transformPointCloud(*pc, *pc, sensorToRobot);

	last_perception_->clear();

	for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it=pc->begin(); it!=pc->end(); ++it)
	{
		if(!std::isnan(it->x))
		{
			double thetaxy, thetaxz;

			thetaxy = atan2(it->y, it->x);
			thetaxz = atan2(it->z, it->x);

			if(fabs(thetaxy)<pointcloudMaxTxy_ && fabs(thetaxz)<pointcloudMaxTxz_ &&
					it->x < pointcloudMaxX_ &&
					it->z > pointcloudMinZ_ && it->z < pointcloudMaxZ_)
				last_perception_->push_back(*it);
		}
	}




	/*











	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

	pcl_conversions::toPCL(*cloud_in, *cloud);

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (res_, res_, res_);
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
*/
	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	//ROS_DEBUG("Pointcloud perception done (%zu pts total, %f sec)", last_perception_->size(), total_elapsed);

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

	return;
}

bool
WmLocalization::validParticlePosition(float x, float y)
{

	if((map_->getInputCloud()==NULL)||(map_->getInputCloud()->empty()))
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
	searchPointw.z = res_*4.0f;

	infloor = map_->radiusSearch(searchPointf, res_*2.0f, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0;
	inwall = map_->radiusSearch(searchPointw, res_*2.0f, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0;

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
	p.p_ = 1.0 / ((float) particles_.size());

	do {
		x = ((float) rand() / (float) RAND_MAX) * (map_max_x_-map_min_x_) + map_min_x_;
		y = ((float) rand() / (float) RAND_MAX) * (map_max_y_-map_min_y_) + map_min_y_;
	} while (!validParticlePosition(x,y));

	t = normalizePi(((float) rand() / (float) RAND_MAX) * 2.0 * M_PI);

	p.coord_.position.x = x;
	p.coord_.position.y = y;
	p.coord_.position.z = 0.0;

	tf::Quaternion q;
	q.setEuler(0.0, 0.0, t);

	p.coord_.orientation.x = q.x();
	p.coord_.orientation.y = q.y();
	p.coord_.orientation.z = q.z();
	p.coord_.orientation.w = q.w();
}

void
WmLocalization::resetParticleNear(Particle& p2reset, const Particle& pref)
{
	float x, y;


	//do {
		x = pref.coord_.position.x + randn(0.0, 0.05);//normalX(generator);
		y = pref.coord_.position.y + randn(0.0, 0.05);//normalY(generator);
	//} while (!validParticlePosition(x,y));

	tf::Quaternion q(pref.coord_.orientation.x, pref.coord_.orientation.y,
			pref.coord_.orientation.z, pref.coord_.orientation.w);

	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	float newt = normalizePi(yaw + randn(0.0, 0.2));

	setParticle(p2reset, x, y, newt);

	p2reset.p_ = pref.p_;//1.0/(float)particles_.size();
}

void
WmLocalization::resetParticles()
{
	ros::WallTime startTime = ros::WallTime::now();

	for (int i = 0; i < particles_.size(); i++)
		resetParticleUniform(particles_[i]);

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
	bool publishPose = posecov_pub_.getNumSubscribers() > 0;

	if(publishPose)
	{
		geometry_msgs::PoseWithCovarianceStamped pose2send;

		pose2send.header.frame_id = "/map";
		pose2send.header.stamp = ros::Time::now();
		pose2send.pose = pose_;

		posecov_pub_.publish(pose2send);
	}
}

void
WmLocalization::publishPose(const ros::Time& rostime)
{
	bool publishPose = pose_pub_.getNumSubscribers() > 0;

	if(publishPose)
	{
		geometry_msgs::PoseStamped pose2send;

		pose2send.header.frame_id = "/map";
		pose2send.header.stamp = ros::Time::now();
		pose2send.pose = pose_.pose;

		pose_pub_.publish(pose2send);
	}
}

void
WmLocalization::publishParticles(const ros::Time& rostime)
{
	bool publishParticles = particles_pub_.getNumSubscribers() > 0;

	if (publishParticles){
		geometry_msgs::PoseArray paray;

		paray.header.frame_id = "/map";
		paray.header.stamp = ros::Time::now();

		for (int i = 0; i < particles_.size(); i++)
			paray.poses.push_back(particles_[i].coord_);

		particles_pub_.publish(paray);
	}
}

void
WmLocalization::publishPerception(const ros::Time& rostime)
{
	size_t perceptionSize = last_perception_->size();

	if (perceptionSize <= 1){
		ROS_WARN("Nothing to publish, perception is empty");
		return;
	}

	bool publishPointCloud = perception_pub_.getNumSubscribers() > 0;

	// finish pointcloud:
	if (publishPointCloud){
		sensor_msgs::PointCloud2 cloud;
		pcl::toROSMsg (*last_perception_, cloud);
		cloud.header.frame_id = baseFrameId_;
		cloud.header.stamp = rostime;
		perception_pub_.publish(cloud);
	}
}


void
WmLocalization::step()
{

	ROS_DEBUG("Step()");
	ros::WallTime startTime = ros::WallTime::now();
	if((map_->getInputCloud()==NULL)||(map_->getInputCloud()->empty()))
	{
		ROS_DEBUG("Doing nothing on this step");
		return;
	}



	if(doResetParticles_)
	{
		ROS_DEBUG("Resetting");
		resetParticles();
		doResetParticles_ = false;
	}

	ROS_DEBUG("Predict");
	predict();

	ROS_DEBUG("Correct");
	correct();

	ROS_DEBUG("Normlize");
	normalize();

	ROS_DEBUG("Reseed");
	reseed();

	ROS_DEBUG("Updating Pos");
	updatePos();

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Localization done (%f sec)", total_elapsed);

	ROS_DEBUG("Publishing all");
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
			tfListener_.lookupTransform("/odom", "base_footprint",
					ros::Time(0), odom2bf);

		} catch (tf::TransformException & ex) {
			ROS_WARN("%s", ex.what());
		}

		desp = last2odom_ * odom2bf;

		last2odom_.setOrigin(odom2bf.inverse().getOrigin());
		last2odom_.setRotation(odom2bf.inverse().getRotation());


		for (int i = 0; i < particles_.size(); i++) {
			tf::Transform part, parttf;

			part.setOrigin(
					tf::Vector3(particles_[i].coord_.position.x,
							particles_[i].coord_.position.y,
							particles_[i].coord_.position.z));
			part.setRotation(
					tf::Quaternion(particles_[i].coord_.orientation.x,
							particles_[i].coord_.orientation.y,
							particles_[i].coord_.orientation.z,
							particles_[i].coord_.orientation.w));

			tf::Transform noise = genNoise(desp);

			parttf = part * desp * noise;

			particles_[i].coord_.position.x = parttf.getOrigin().getX();
			particles_[i].coord_.position.y = parttf.getOrigin().getY();
			particles_[i].coord_.position.z = parttf.getOrigin().getZ();
			particles_[i].coord_.orientation.x = parttf.getRotation().getX();
			particles_[i].coord_.orientation.y = parttf.getRotation().getY();
			particles_[i].coord_.orientation.z = parttf.getRotation().getZ();
			particles_[i].coord_.orientation.w = parttf.getRotation().getW();

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

	npoints = map_->radiusSearch(searchPoint, res_, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	int inc=2;
	for(int i=0; i<npoints; i=i+inc)
	{
		pcl::PointXYZHSV hsvA, hsvB;
		float diffH, diffS, diffV;

		pcl::PointXYZRGB point = map_->getInputCloud()->points[pointIdxRadiusSearch[i]];

		PointXYZRGBtoXYZHSV(searchPoint, hsvA);
		PointXYZRGBtoXYZHSV(point, hsvB);

		diffH = fabs(normalizePi(toRadians(hsvA.h - 180.0) - toRadians(hsvB.h - 180.0))/M_PI);

		diffS = fabs(hsvA.s - hsvB.s);
		diffV = fabs(hsvA.v - hsvB.v);

		float vd = (res_-sqrt(pointRadiusSquaredDistance[0]))/res_;

		ret += (KO * vd + KH * (1.0 - diffH) + KS * (1.0 - diffS)+ KV * (1.0 - diffV))*((res_-pointRadiusSquaredDistance[i])/res_);

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

	W2H.setOrigin(tf::Vector3(p.coord_.position.x, p.coord_.position.y, p.coord_.position.z));

	W2H.setRotation( tf::Quaternion(p.coord_.orientation.x, p.coord_.orientation.y,
			p.coord_.orientation.z, p.coord_.orientation.w));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr testp(new pcl::PointCloud<pcl::PointXYZRGB>);

	float temp = 0.0;;
	//p.p = 0.0f;

	for (int i = 0; i < correctpoints_; i++)
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

	p.p_ = 0.8*p.p_ + 0.2 * (temp/correctpoints_);

	if(p.p_>1.0) p.p_=1.0;

}

void
WmLocalization::correct()
{
	ros::WallTime startTime = ros::WallTime::now();

	if(last_perception_->empty())
		return;


	//ToDo: Use keypoints instead of random ones
	std::vector<pcl::PointXYZRGB> testpoints(correctpoints_);
	for (int i = 0; i < correctpoints_; i++)
	{
		int idx = rand() % last_perception_->size();
		testpoints[i] = last_perception_->at(idx);
	}

	for(int j=0; j<particles_.size(); j++)
		updateParticle(particles_[j], testpoints);

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Correction done (%f sec)", total_elapsed);

}
void
WmLocalization::normalize() {

	float sum = 0.0;
	float factor;

	for (int i = 0; i < particles_.size(); i++)
		sum = sum + particles_[i].p_;

	if(sum>0.0)
		factor = 1.0 / sum;
	else
		factor = 1.0;

	for (int i = 0; i < particles_.size(); i++)
		particles_[i].p_ = particles_[i].p_ * factor;

}


void
WmLocalization::reseed()
{
	std::sort(particles_.begin(), particles_.end());
	std::reverse(particles_.begin(), particles_.end());

	float area_unc = sqrt(pose_.covariance[0]) * sqrt(pose_.covariance[1 * 6 + 1]);

	if(area_unc>3.0)
	{
		int new_size = particles_.size() * 1.5;
		if(new_size>numparticles_max_) new_size = numparticles_max_;

		int add_p = new_size-particles_.size();

		for(int i=0; i<add_p;i++)
			particles_.push_back(Particle());
	}else if(area_unc<0.1)
	{
		int new_size = particles_.size() / 1.5;
		if(new_size<numparticles_min_) new_size = numparticles_min_;
		int del_p = particles_.size()-new_size;

		particles_.erase(particles_.end()-del_p, particles_.end());
	}

	int idx_25 = particles_.size()*25/100;
	int idx_50 = particles_.size()*50/100;
	int idx_75 = particles_.size()*75/100;

	for(int i=idx_50; i<idx_75;i++)
		resetParticleUniform(particles_[i]);

	int c=0;
	for(int i=idx_75; i<particles_.size();i++)
	{
		resetParticleNear(particles_[i], particles_[c]);
		c = (c+1)%idx_25;
	}



}


void WmLocalization::updatePos()
{
	pose_.pose = particles_[0].coord_;

	geometry_msgs::Pose error;

	error.position.x = error.position.y = error.position.z;
	error.orientation =  tf::createQuaternionMsgFromRollPitchYaw (0.0, 0.0, 0.0);

	float p_mean = 0.0f;

	int sgnificant_part = particles_.size()*50/100;
	for(int i=0; i<sgnificant_part; i++)
	{
		error.position.x = error.position.x +
				(particles_[i].coord_.position.x -particles_[0].coord_.position.x) *
				(particles_[i].coord_.position.x -particles_[0].coord_.position.x);

		error.position.y = error.position.y +
				(particles_[i].coord_.position.y -particles_[0].coord_.position.y) *
				(particles_[i].coord_.position.y -particles_[0].coord_.position.y);

		error.position.z = error.position.z +
				(particles_[i].coord_.position.z -particles_[0].coord_.position.z) *
				(particles_[i].coord_.position.z -particles_[0].coord_.position.z);

		error.orientation.x = error.orientation.x +
				(particles_[i].coord_.orientation.x -particles_[0].coord_.orientation.x) *
				(particles_[i].coord_.orientation.x -particles_[0].coord_.orientation.x);

		error.orientation.y = error.orientation.y +
				(particles_[i].coord_.orientation.y -particles_[0].coord_.orientation.y) *
				(particles_[i].coord_.orientation.y -particles_[0].coord_.orientation.y);

		error.orientation.z = error.orientation.z +
				(particles_[i].coord_.orientation.z -particles_[0].coord_.orientation.z) *
				(particles_[i].coord_.orientation.z -particles_[0].coord_.orientation.z);

		error.orientation.w = error.orientation.w +
				(particles_[i].coord_.orientation.w -particles_[0].coord_.orientation.w) *
				(particles_[i].coord_.orientation.w -particles_[0].coord_.orientation.w);

		p_mean = p_mean + particles_[i].p_ * particles_[i].p_;
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
			pose_.covariance[i] = 0.0;

	pose_.covariance[0] = error.position.x;
	pose_.covariance[1 * 6 + 1] =error.position.y;
	pose_.covariance[5 * 6 + 5] = error.position.z;

	ROS_DEBUG("[%ld]\tp: %f\tstdvev x:%f\ty:%f\tz:%f", particles_.size(), p_mean, error.position.x, error.position.y, error.position.z);

}

void WmLocalization::setParticle(Particle &particle, float x, float y, float theta)
{

	particle.coord_.position.x = x;
	particle.coord_.position.y = y;
	particle.coord_.position.z = 0.0;

	tf::Quaternion q;
	q.setEuler(0.0, 0.0, theta);

	particle.coord_.orientation.x = q.x();
	particle.coord_.orientation.y = q.y();
	particle.coord_.orientation.z = q.z();
	particle.coord_.orientation.w = q.w();

}

}

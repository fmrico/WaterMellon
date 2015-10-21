/*
 * WmCommonFrame.cpp
 *
 *  Created on: 19/10/2015
 *      Author: paco
 */

#include <wm_objects/WmObjectTrainer.h>

namespace wm_objects {

WmObjectTrainer::WmObjectTrainer(ros::NodeHandle private_nh_)
: nh_(),
  objectFrameId_("/object"),
  cameraFrameId_("/camera_link"),
  cameraTopicId_("/camera/depth_registered/points"),
  object_(new pcl::PointCloud<pcl::PointXYZRGB>),
  object_octree_(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(128.0f)),
  max_z_(0.5)
{
	ros::NodeHandle private_nh(private_nh_);
	private_nh.param("objectFrameId", objectFrameId_, objectFrameId_);
	private_nh.param("cameraFrameId", cameraFrameId_, cameraFrameId_);
	private_nh.param("cameraTopicId", cameraTopicId_, cameraTopicId_);
	private_nh.param("pointcloud_max_z", max_z_,max_z_);

	pointCloudSub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, cameraTopicId_, 5);
	tfPointCloudSub_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*pointCloudSub_, tfListener_, cameraFrameId_, 5);
	tfPointCloudSub_->registerCallback(boost::bind(&WmObjectTrainer::insertCloudCallback, this, _1));

	objectService_ = nh_.advertiseService("object", &WmObjectTrainer::objectSrv, this);
	objectPub_ =  nh_.advertise<sensor_msgs::PointCloud2>("/object", 1, true);
	object_octree_->setInputCloud(object_);

	initMarkers();
}


void
WmObjectTrainer::initMarkers()
{
	tf::Transform t;
	t.setOrigin(tf::Vector3(-0.1175, -0.079, 0.0));
	t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_["ar_marker_0"] = t;

	t.setOrigin(tf::Vector3(-0.1175, 0.0, 0.0));
	t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_["ar_marker_1"] = t;

	t.setOrigin(tf::Vector3(-0.1175, 0.079, 0.0));
	t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_["ar_marker_2"] = t;

	t.setOrigin(tf::Vector3(0.0, -0.079, 0.0));
	t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_["ar_marker_3"] = t;

	t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_["ar_marker_4"] = t;

	t.setOrigin(tf::Vector3(0.0, 0.079, 0.0));
	t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_["ar_marker_5"] = t;

	t.setOrigin(tf::Vector3(0.1175, -0.079, 0.0));
	t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_["ar_marker_6"] = t;

	t.setOrigin(tf::Vector3(0.1175, 0.0, 0.0));
	t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_["ar_marker_7"] = t;

	t.setOrigin(tf::Vector3(0.1175, 0.079, 0.0));
	t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_["ar_marker_8"] = t;
}

bool
WmObjectTrainer::objectSrv(watermellon::GetObject::Request  &req, watermellon::GetObject::Response &res)
{

	ROS_INFO("Sending object  on service request");

	if (object_->size() <= 1){
		ROS_WARN("Nothing to publish, object is empty");
		return false;
	}

	pcl::toROSMsg (*object_, res.object);
	res.object.header.frame_id = objectFrameId_;
	res.object.header.stamp = ros::Time::now();

	return true;
}


void
WmObjectTrainer::calculateSetMeanStdv(const std::vector<tf::StampedTransform>&set, tf::Vector3& mean, tf::Vector3& stdev)
{
	int c=0;

	mean = tf::Vector3(0.0, 0.0, 0.0);
	stdev = tf::Vector3(0.0, 0.0, 0.0);


	for(std::vector<tf::StampedTransform>::const_iterator it=set.begin(); it!=set.end(); ++it)
	{
		tf::Transform center= it->inverse() * marks2center_[it->frame_id_];

		ROS_DEBUG("NEW MARKS2CENTER (%lf, %lf, %lf) (%lf, %lf, %lf, %lf)",
				center.getOrigin().x(), center.getOrigin().y(), center.getOrigin().z(),
				center.getRotation().x(), center.getRotation().y(), center.getRotation().z(), center.getRotation().w());

		mean = mean + center.getOrigin();
		c++;
	}

	mean = mean/c;

	for(std::vector<tf::StampedTransform>::const_iterator it=set.begin(); it!=set.end(); ++it)
	{
		tf::Transform center= it->inverse() * marks2center_[it->frame_id_];
		stdev = stdev + (mean-center.getOrigin())*(mean-center.getOrigin());
	}

	stdev = stdev /c;
}

void
WmObjectTrainer::getValidMarks(std::vector<tf::StampedTransform>& marks, const ros::Time& stamp)
{
	marks.clear();
	for(int i=0; i<MAX_MARKS; i++)
	{
		tf::StampedTransform mark;
		std::stringstream mark_frame;
		mark_frame<<"ar_marker_"<<i;
		try {
			tfListener_. lookupTransform(mark_frame.str(), cameraFrameId_, stamp, mark);
			ROS_DEBUG("Frame [%s] found", mark.frame_id_.c_str());

			if(marks2center_.find(mark.frame_id_) != marks2center_.end())
				marks.push_back(mark);

		} catch(tf::TransformException& ex){}
	}

}

void
WmObjectTrainer::getBestTransform(const std::vector<tf::StampedTransform>& marks, tf::Transform& trans, double& stdev)
{
	std::string best;
	double minstd=100.0;

	Combinations<tf::StampedTransform> c(marks, MIN_VALID_MARKS);
	for(Combinations<tf::StampedTransform>::iterator it=c.begin(); it!=c.end(); ++it)
	{

		ROS_DEBUG("media de %s, %s, %s, %s", (*it)[0].frame_id_.c_str(), (*it)[1].frame_id_.c_str(),
				(*it)[2].frame_id_.c_str(), (*it)[3].frame_id_.c_str());

		tf::Vector3 mean;
		tf::Vector3 stdev;
		calculateSetMeanStdv(*it, mean, stdev);

		if(((stdev[0]+stdev[1]+stdev[2])/3.0)<minstd)
		{
			best="[";

			trans.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
			trans.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 0.0));

			minstd = (stdev[0]+stdev[1]+stdev[2])/3.0;
			int c=0;
			for(std::vector<tf::StampedTransform>::const_iterator it2=it->begin(); it2!=it->end(); ++it2)
			{
				tf::Transform mark2center;

				mark2center = it2->inverse() * marks2center_[it2->frame_id_];

				best=best+it2->frame_id_+" ,";
				if(c==0)
				{
					trans.setOrigin(mark2center.getOrigin());
					trans.setRotation(mark2center.getRotation());
				}else
				{
					trans.setOrigin(trans.getOrigin() + mark2center.getOrigin());
					trans.setRotation(trans.getRotation() + mark2center.getRotation());
				}
				c++;
			}

			trans.setOrigin(trans.getOrigin()/c);
			trans.setRotation(trans.getRotation()/c);
			best=best+"]";
		}

		ROS_DEBUG("NEW WINNER (%lf, %lf, %lf) (%lf, %lf, %lf, %lf)",
				trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z(),
				trans.getRotation().x(), trans.getRotation().y(), trans.getRotation().z(), trans.getRotation().w());

		//ROS_INFO("(%lf, %lf, %lf)  (%lf, %lf, %lf)",  mean[0], mean[1], mean[2], stdev[0], stdev[1], stdev[2]);
	}

	//ROS_INFO("BEST = %s", best.c_str());

	stdev = minstd;
}


bool
WmObjectTrainer::updateObjectFrame(const ros::Time& stamp, tf::StampedTransform& m2c)
{
	std::vector<tf::StampedTransform> valids;

	getValidMarks(valids, stamp);
	if(valids.size()<4)
	{
		//ROS_INFO("No enough marks detected [%zu]", valids.size());
		return false;
	}

	tf::Transform media;
	double stdev;
	getBestTransform(valids, media, stdev);

	double m_tolerance2 = 0.006*0.006;
	if(stdev<m_tolerance2)
	{
		//ROS_INFO("TRANSFORM ACCEPTED ");
		m2c.child_frame_id_ = objectFrameId_;
		m2c.frame_id_ = cameraFrameId_;
		m2c.stamp_ = ros::Time::now();

		m2c.setOrigin(media.getOrigin());
		m2c.setRotation(media.getRotation());

		ROS_DEBUG("NEW FINAL (%lf, %lf, %lf) (%lf, %lf, %lf, %lf)",
				m2c.getOrigin().x(), m2c.getOrigin().y(), m2c.getOrigin().z(),
				m2c.getRotation().x(), m2c.getRotation().y(), m2c.getRotation().z(), m2c.getRotation().w());

		try
		{
			tfBroadcaster_.sendTransform(m2c);

		}catch(tf::TransformException & ex)
		{
			ROS_WARN("WmObjectTrainer::updateObjectFrame %s",ex.what());
		}
		return true;
	}

	//ROS_INFO("TRANSFORM REJECTED ");
	return false;

}

bool
WmObjectTrainer::validNewPoint(const pcl::PointXYZRGB& point)
{
	if(object_octree_->getInputCloud()->size()<1) return true;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	int same_pos = object_octree_->radiusSearch(point, 0.001, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	if(same_pos==0)
		return true;

	std::vector<int>::iterator it;
	for(it=pointIdxRadiusSearch.begin(); it!=pointIdxRadiusSearch.end(); ++it)
	{
		int r,g,b;
		r= object_octree_->getInputCloud()->at(*it).r;
		g= object_octree_->getInputCloud()->at(*it).g;
		b= object_octree_->getInputCloud()->at(*it).b;

		if((abs(point.r-r)<20) && (abs(point.g-g)<20) && (abs(point.b-b)<20))
			return false;
	}

	return true;

}

void
WmObjectTrainer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
	ros::WallTime startTime = ros::WallTime::now();

	tf::StampedTransform cameraToObjectTf;
	if(!updateObjectFrame(cloud_in->header.stamp, cameraToObjectTf))
	{
		ROS_INFO("WmObjectTrainer: No points added due to rejected transform");
		return;
	}
	tf::StampedTransform sensorToCameraTf;
	try {
		tfListener_.lookupTransform(cameraFrameId_, cloud_in->header.frame_id, cloud_in->header.stamp, sensorToCameraTf);
	} catch(tf::TransformException& ex){
		ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}

	Eigen::Matrix4f sensorToCamera;
	pcl_ros::transformAsMatrix(sensorToCameraTf, sensorToCamera);

	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
	pcl_conversions::toPCL(*cloud_in, *cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_camera(new pcl::PointCloud<pcl::PointXYZRGB>); // input cloud for filtering and ground-detection
	pcl::fromPCLPointCloud2 (*cloud, *pc);

	pcl::transformPointCloud(*pc, *pc_camera, sensorToCamera);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_object(new pcl::PointCloud<pcl::PointXYZRGB>); // input cloud for filtering and ground-detection
	Eigen::Matrix4f CameraToObject;
	pcl_ros::transformAsMatrix(cameraToObjectTf.inverse(), CameraToObject);
	pcl::transformPointCloud(*pc_camera, *pc_object, CameraToObject);


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_in(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it=pc_object->begin(); it!=pc_object->end(); ++it)
	{
		if(std::isnan(it->x) || std::isinf(it->x) ||
				(fabs(it->x)>0.08 || fabs(it->y)>0.12 || it->z>0.30 || it->z<0.01) ||
				((it->z < 0.01) && ((it->r>200 && it->g>200 && it->b>200) ||
						(it->r<10 && it->g<10 && it->b<10))))
			continue;

		pc_in->push_back(*it);
	}

	static bool start=true;
	if( false && !start && pc_in->size()>0)
	{

		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

		icp.setMaxCorrespondenceDistance (0.002);
		icp.setMaximumIterations (50);
		icp.setTransformationEpsilon (1e-8);
		icp.setEuclideanFitnessEpsilon (1);

		icp.setInputSource(object_);
		icp.setInputTarget(pc_in);

		pcl::PointCloud<pcl::PointXYZRGB> final;
		icp.align(final);
		Eigen::Matrix4f correction = icp.getFinalTransformation();
		correction = correction;//.inverse();
		pcl::transformPointCloud(*pc_in, *pc_in, correction);
	}
	start=false;

	for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it=pc_in->begin(); it!=pc_in->end(); ++it)
	{
		if(!validNewPoint(*it)) continue;
		object_->push_back(*it);
		object_octree_->addPointToCloud (*it, object_);
	}


	if (objectPub_.getNumSubscribers() > 0){
		sensor_msgs::PointCloud2 cloud_out;

		pcl::toROSMsg (*object_, cloud_out);

		cloud_out.header.frame_id = objectFrameId_;
		cloud_out.header.stamp = ros::Time::now();

		objectPub_.publish(cloud_out);
	}
	ROS_INFO("WmObjectTrainer: Object points: %zu", object_->size());

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Pointcloud insertion in object done (%zu pts total, %f sec)", object_->size(), total_elapsed);

	return;
}

WmObjectTrainer::~WmObjectTrainer() {
	// TODO Auto-generated destructor stub
}

} /* namespace wm_objects */

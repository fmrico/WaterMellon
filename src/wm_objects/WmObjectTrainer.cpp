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
	marks2center_[0].setOrigin(tf::Vector3(-0.1175, -0.079, 0.0));
	marks2center_[0].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_[1].setOrigin(tf::Vector3(-0.1175, 0.0, 0.0));
	marks2center_[1].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_[2].setOrigin(tf::Vector3(-0.1175, 0.079, 0.0));
	marks2center_[2].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_[3].setOrigin(tf::Vector3(0.0, -0.079, 0.0));
	marks2center_[3].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_[4].setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	marks2center_[4].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_[5].setOrigin(tf::Vector3(0.0, 0.079, 0.0));
	marks2center_[5].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_[6].setOrigin(tf::Vector3(0.1175, -0.079, 0.0));
	marks2center_[6].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_[7].setOrigin(tf::Vector3(0.1175, 0.0, 0.0));
	marks2center_[7].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	marks2center_[8].setOrigin(tf::Vector3(0.1175, 0.079, 0.0));
	marks2center_[8].setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
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

bool
WmObjectTrainer::updateObjectFrame(const ros::Time& stamp, tf::StampedTransform& m2c)
{
	int valid=0;
	tf::StampedTransform marks[MAX_MARKS];

	bool valids[MAX_MARKS];

	for(int i=0; i<MAX_MARKS; i++) valids[i]=false;

	for(int i=0; i<MAX_MARKS; i++)
	{
		std::stringstream mark_frame;
		mark_frame<<"ar_marker_"<<i;
		try {
			tfListener_. lookupTransform(mark_frame.str(), cameraFrameId_, stamp, marks[i]);
			ROS_DEBUG("Frame [%s] found", marks[i].frame_id_.c_str());
			valid++;
			valids[i]=true;
		} catch(tf::TransformException& ex){
			//ROS_ERROR_STREAM( "No transform: " << ex.what() << ", quitting callback");
		}
	}


	if(valid>=MIN_VALID_MARKS)
	{
		tf::Transform center[MAX_MARKS];
		tf::Transform media;

		media.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		media.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 0.0));

		int c=0;

		for(int i=0; i<MAX_MARKS; i++)
		{
			if(valids[i])
			{
				center[i]= marks[i].inverse() * marks2center_[i] ;

				ROS_DEBUG("[%d] (%lf, %lf, %lf) (%lf, %lf, %lf, %lf)", i,
						center[i].getOrigin().x(), center[i].getOrigin().y(), center[i].getOrigin().z(),
						center[i].getRotation().x(), center[i].getRotation().y(), center[i].getRotation().z(), center[i].getRotation().w());

				if(c==0)
				{
					media.setOrigin(center[i].getOrigin());
					media.setRotation(center[i].getRotation());
				}else
				{
					media.setOrigin(media.getOrigin() + center[i].getOrigin());
					media.setRotation(media.getRotation() + center[i].getRotation());
				}
				c++;
			}
		}

		media.setOrigin(media.getOrigin()/c);
		media.setRotation(media.getRotation()/c);


		tf::Vector3 desv(0.0, 0.0, 0.0);
		for(int i=0; i<MAX_MARKS; i++)
			if(valids[i]) desv = desv + (media.getOrigin()-center[i].getOrigin())*(media.getOrigin()-center[i].getOrigin());
		desv = desv/c;
		double m_tolerance2 = 0.005*0.005;

		ROS_DEBUG("MEDIA de %d (%lf, %lf, %lf) desv (%lf, %lf, %lf) [%lf]", c,
				media.getOrigin().x(), media.getOrigin().y(), media.getOrigin().z(),
				desv.x(), desv.y(),desv.z(), m_tolerance2);



		if(desv.x()<m_tolerance2 && desv.y()<m_tolerance2 && desv.z()<m_tolerance2)
		{
			ROS_DEBUG("TRANSFORM ACCEPTED");

			m2c.child_frame_id_ = objectFrameId_;
			m2c.frame_id_ = cameraFrameId_;
			m2c.stamp_ = ros::Time::now();

			m2c.setOrigin(media.getOrigin());
			m2c.setRotation(media.getRotation());

			try
			{
				tfBroadcaster_.sendTransform(m2c);

			}catch(tf::TransformException & ex)
			{
				ROS_WARN("WmObjectTrainer::updateObjectFrame %s",ex.what());
			}
			return true;
		}else
		{
			ROS_DEBUG("TRANSFORM REJECTED");

		}

	}
	ROS_DEBUG("TRANSFORM REJECTED %d", valid);
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
		return;
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
	if(!start && pc_in->size()>0)
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
		correction = correction.inverse();
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

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Pointcloud insertion in object done (%zu pts total, %f sec)", object_->size(), total_elapsed);

	return;
}

WmObjectTrainer::~WmObjectTrainer() {
	// TODO Auto-generated destructor stub
}

} /* namespace wm_objects */

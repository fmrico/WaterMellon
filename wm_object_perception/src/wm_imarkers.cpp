/*
 * wm_imarkers.cpp
 *
 *  Created on: 05/09/2015
 *      Author: paco
 */


#include <wm_object_perception/wm_imarkers.h>


namespace wm_markers {

WmIMarkers::WmIMarkers()
:
					server_("wm_modeler"),
					frame_id_("/camera_link"),
					object_(new pcl::PointCloud<pcl::PointXYZRGB>)
{

	geometry_msgs::Pose pose_min, pose_max;

	pose_min.position.z = -0.25;
	pose_min.position.x = -0.10;
	pose_min.position.y = -0.10;
	initMarker(min_marker_, "object_box_min", "Min 3-DOF Control", pose_min);
	pose_max.position.z = 0.0;
	pose_max.position.x = 0.10;
	pose_max.position.y = 0.10;

	initMarker(max_marker_, "object_box_max", "Max 3-DOF Control", pose_max);

	server_.applyChanges();


	ObjectPub_ = m_nh.advertise<sensor_msgs::PointCloud2>("object", 1, true);

	m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
	m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, frame_id_, 5);
	m_tfPointCloudSub->registerCallback(boost::bind(&WmIMarkers::insertCloudCallback, this, _1));


	//  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
	//    ros::console::notifyLoggerLevelsChanged();

}

WmIMarkers::~WmIMarkers()
{
	server_.clear();
}

void
WmIMarkers::initMarker(visualization_msgs::InteractiveMarker &marker,  std::string name, std::string desc, geometry_msgs::Pose pose)
{

	marker.header.frame_id =frame_id_;
	marker.header.stamp=ros::Time::now();
	marker.name = name;
	marker.description = desc;

	// create a grey box marker
	visualization_msgs::Marker box_marker;
	box_marker.type = visualization_msgs::Marker::CUBE;
	box_marker.scale.x = 0.45;
	box_marker.scale.y = 0.45;
	box_marker.scale.z = 0.45;
	box_marker.color.r = 0.5;
	box_marker.color.g = 0.5;
	box_marker.color.b = 0.5;
	box_marker.color.a = 1.0;
	box_marker.pose = pose;

	// create a non-interactive control which contains the box
	visualization_msgs::InteractiveMarkerControl box_control;
	box_control.always_visible = true;
	box_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
	box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;

	box_control.markers.push_back( box_marker );

	// add the control to the interactive marker
	marker.controls.push_back( box_control );

	visualization_msgs::InteractiveMarkerControl move_x_control;
	move_x_control.name = "move_x";
	move_x_control.orientation.x = 1;
	move_x_control.orientation.y = 0;
	move_x_control.orientation.z = 0;
	move_x_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	marker.controls.push_back(move_x_control);

	visualization_msgs::InteractiveMarkerControl move_y_control;
	move_y_control.name = "move_y";
	move_y_control.orientation.w = 1;
	move_y_control.orientation.x = 0;
	move_y_control.orientation.y = 1;
	move_y_control.orientation.z = 0;

	move_y_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	marker.controls.push_back(move_y_control);

	visualization_msgs::InteractiveMarkerControl move_z_control;
	move_z_control.name = "move_z";
	move_z_control.orientation.w = 1;
	move_z_control.orientation.x = 0;
	move_z_control.orientation.y = 0;
	move_z_control.orientation.z = 1;
	move_z_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	marker.controls.push_back(move_z_control);


	server_.insert(marker, boost::bind(&WmIMarkers::processFeedback, this, _1));
}

void
WmIMarkers::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ROS_INFO_STREAM( feedback->marker_name << " is now at "
			<< feedback->pose.position.x << ", " << feedback->pose.position.y
			<< ", " << feedback->pose.position.z );

	visualization_msgs::InteractiveMarker min, max;
	server_.get("object_box_min", min);
	server_.get("object_box_max", max);

	if(feedback->marker_name  == "object_box_min")
	{
		if(min.pose.position.x>max.pose.position.x) max.pose.position.x=min.pose.position.x;
		if(min.pose.position.y>max.pose.position.y) max.pose.position.y=min.pose.position.y;
		if(min.pose.position.z>max.pose.position.z) max.pose.position.z=min.pose.position.z;
		server_.setPose("object_box_max", max.pose);
	}
	if(feedback->marker_name  == "object_box_max")
	{
		if(min.pose.position.x>max.pose.position.x) min.pose.position.x=max.pose.position.x;
		if(min.pose.position.y>max.pose.position.y) min.pose.position.y=max.pose.position.y;
		if(min.pose.position.z>max.pose.position.z) min.pose.position.z=max.pose.position.z;
		server_.setPose("object_box_min", min.pose);
	}

	server_.applyChanges();

}


void
WmIMarkers::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{


	ros::WallTime startTime = ros::WallTime::now();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>); // input cloud for filtering and ground-detection
	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
	pcl_conversions::toPCL(*cloud_in, *cloud);
	pcl::fromPCLPointCloud2 (*cloud, *pc);

	tf::StampedTransform sensorToWorldTf;
	try {
		m_tfListener.lookupTransform(frame_id_, cloud_in->header.frame_id, cloud_in->header.stamp, sensorToWorldTf);
	} catch(tf::TransformException& ex){
		ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}

	Eigen::Matrix4f sensorToWorld;
	pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

	visualization_msgs::InteractiveMarker min, max;
	server_.get("object_box_min", min);
	server_.get("object_box_max", max);


	pcl::PassThrough<pcl::PointXYZRGB> passX, passY, passZ;
	passX.setFilterFieldName("x");
	passX.setFilterLimits(min.pose.position.x, max.pose.position.x);
	passY.setFilterFieldName("y");
	passY.setFilterLimits(min.pose.position.y, max.pose.position.y);
	passZ.setFilterFieldName("z");
	passZ.setFilterLimits(min.pose.position.z, max.pose.position.z);

	ROS_DEBUG_STREAM("["<<min.pose.position.x<<"-"<<max.pose.position.x<<"]\t["<<
			min.pose.position.y<<"-"<<max.pose.position.y<<"]\t["<<
			min.pose.position.z<<"-"<<max.pose.position.z<<"]"<<std::endl);

	// directly transform to map frame:
	pcl::transformPointCloud(*pc, *pc, sensorToWorld);

	// just filter height range:
	passX.setInputCloud(pc->makeShared());
	passX.filter(*pc);
	passY.setInputCloud(pc->makeShared());
	passY.filter(*pc);
	passZ.setInputCloud(pc->makeShared());
	passZ.filter(*pc);

	//object_ = object_ + *pc;



	/*ROS_INFO_STREAM( "Publishing "<<cloud->in
				<< feedback->pose.position.x << ", " << feedback->pose.position.y
				<< ", " << feedback->pose.position.z );*/
	publishObject(cloud_in->header.stamp);

	return;
}

void
WmIMarkers::publishObject(const ros::Time& rostime)
{

	size_t objSize = object_->size();

	if (objSize <= 1){
		ROS_WARN("Nothing to publish, object data is empty");
		return;
	}

	bool publishPointCloud = ObjectPub_.getNumSubscribers() > 0;

	// finish pointcloud:
	if (publishPointCloud){
		sensor_msgs::PointCloud2 cloud;
		pcl::toROSMsg (*object_, cloud);
		cloud.header.frame_id = frame_id_;
		cloud.header.stamp = rostime;
		ObjectPub_.publish(cloud);
	}

}


}

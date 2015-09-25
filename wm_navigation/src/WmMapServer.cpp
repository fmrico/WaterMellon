/*
 * mapper.cpp
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */

#include "wm_navigation/WmMapServer.h"

namespace wm_map_server{


WmMapServer::WmMapServer(ros::NodeHandle private_nh_)
: m_nh(),
  m_pointcloudMinZ(-std::numeric_limits<double>::max()),
  m_pointcloudMaxZ(std::numeric_limits<double>::max()),
  m_res(0.05),
  m_mapping(true),
  m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
  map(new pcl::PointCloud<pcl::PointXYZRGB>),
  octree(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(128.0f))
{
	ros::NodeHandle private_nh(private_nh_);
	private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
	private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
	private_nh.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
	private_nh.param("pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
  private_nh.param("resolution", m_res, m_res);
  private_nh.param("mapping", m_mapping, m_mapping);

  m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("map", 1, true); //latched

	m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
	m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
	m_tfPointCloudSub->registerCallback(boost::bind(&WmMapServer::insertCloudCallback, this, _1));

  m_mapService = m_nh.advertiseService("map", &WmMapServer::mapSrv, this);


  octree->setInputCloud(map);
}

WmMapServer::~WmMapServer() {

}

bool
WmMapServer::mapSrv(wm_navigation::GetMap::Request  &req, wm_navigation::GetMap::Response &res)
{

  ROS_INFO("Sending map  on service request");

  if (map->size() <= 1){
    ROS_WARN("Nothing to publish, map is empty");
    return false;
  }

  pcl::toROSMsg (*map, res.map);
  res.map.header.frame_id = m_worldFrameId;
  res.map.header.stamp = ros::Time::now();

  return true;
}


void
WmMapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
  ros::WallTime startTime = ros::WallTime::now();

  if(!m_mapping)
  {

    publishMap(cloud_in->header.stamp);
    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG("Pointcloud insertion in map not performed:  (%f sec)",total_elapsed);

    return;
  }

  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  pcl_conversions::toPCL(*cloud_in, *cloud);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (m_res, m_res, m_res);
  sor.filter (*cloud_filtered);

  ROS_DEBUG("Downgrade %zu -> %zu pts)", cloud->data.size(), cloud_filtered->data.size());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>); // input cloud for filtering and ground-detection
  pcl::fromPCLPointCloud2 (*cloud_filtered, *pc);

  tf::StampedTransform sensorToWorldTf;
	try {
		m_tfListener.lookupTransform(m_worldFrameId, cloud_in->header.frame_id, cloud_in->header.stamp, sensorToWorldTf);
	} catch(tf::TransformException& ex){
		ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}

	Eigen::Matrix4f sensorToWorld;
	pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

	// directly transform to map frame:
	pcl::transformPointCloud(*pc, *pc, sensorToWorld);

	// just filter height range:
	pass.setInputCloud(pc->makeShared());
	pass.filter(*pc);

  int c=0;

  ROS_DEBUG("Input Pointcloud %zu",  pc->size());

  bool newPoint = false;
  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it=pc->begin(); it!=pc->end(); ++it)
	   if(validNewPoint(*it))
     {
        map->push_back(*it);
        octree->addPointToCloud (*it, map);
        //kdtree->setInputCloud(map);
        c++;
     }

	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Pointcloud insertion in map done (%d new points, %zu pts total, %f sec)", c, map->size(), total_elapsed);

  //std::cerr<<"*";
	publishMap(cloud_in->header.stamp);

  return;
}

bool
WmMapServer::validNewPoint(const pcl::PointXYZRGB& point)
{
  bool ret;

  if(octree->getInputCloud()->size()<1) return true;

  std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

  ret = octree->radiusSearch(point, m_res, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0;

  return ret;

}

bool
WmMapServer::openFile(const std::string& filename)
{

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *map) == -1)
	{
		ROS_ERROR ("Couldn't read file %s", filename.c_str());
		return false;
	}

	return true;
}

void WmMapServer::publishMap(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();

  size_t mapSize = map->size();

  if (mapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  bool publishPointCloud = m_pointCloudPub.getNumSubscribers() > 0;

  // finish pointcloud:
  if (publishPointCloud){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg (*map, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp = rostime;
    m_pointCloudPub.publish(cloud);
  }

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);

}

}

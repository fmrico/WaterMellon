#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/impl/point_types.hpp"
#include "pcl/point_types_conversion.h"
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <pcl/io/pcd_io.h>

#include <wm_navigation/GetMap.h>


#define USAGE "\nUSAGE: wm_map_saver <mapfile.pcd>\n"

using namespace std;


class MapSaver{
public:
  MapSaver(const std::string& mapname)
  {
    ros::NodeHandle n;
    std::string servname = "map";

    ROS_INFO("Requesting the map from %s...", n.resolveName(servname).c_str());

    wm_navigation::GetMap::Request req;
    wm_navigation::GetMap::Response resp;

    while(n.ok() && !ros::service::call(servname, req, resp))
    {
      ROS_WARN("Request to %s failed; trying again...", n.resolveName(servname).c_str());
      usleep(1000000);
    }

    if (n.ok()){

      pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>); // input cloud for filtering and ground-detection

      pcl_conversions::toPCL(resp.map, *cloud);
      pcl::fromPCLPointCloud2 (*cloud, *pc);

      if (pcl::io::savePCDFileASCII<pcl::PointXYZRGB> (mapname, *pc) != -1)
      {
        ROS_INFO("Map saved (%zu pts)", cloud->data.size());
      }else
      {
         	ROS_ERROR ("Couldn't write file %s", mapname.c_str());
      }

    }

  }

};

int main(int argc, char** argv){
  ros::init(argc, argv, "wm_map_saver");
  std::string mapFilename("");

  if (argc == 2)
    mapFilename = std::string(argv[1]);
  else{
    ROS_ERROR("%s", USAGE);
    exit(1);
  }

  try{
    MapSaver ms(mapFilename);
  }catch(std::runtime_error& e){
    ROS_ERROR("wm_map_saver exception: %s", e.what());
    exit(2);
  }

  exit(0);
}

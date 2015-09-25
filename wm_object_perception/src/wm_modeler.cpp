#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

#include <wm_object_perception/wm_imarkers.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "wm_modeler");
  ros::NodeHandle nh;

  wm_markers::WmIMarkers marker_server;

  ros::Rate loop_rate(20);

  while ( ros::ok())
  {
	  ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;

}

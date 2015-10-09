/*
 * WmGlobalNavigationAS.h
 *
 *  Created on: 08/10/2015
 *      Author: paco
 */

#ifndef WMGLOBALNAVIGATIONAS_H_
#define WMGLOBALNAVIGATIONAS_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <wm_navigation/wm_navigation_alAction.h>

#include <wm_navigation/WmGlobalNavigation.h>

namespace wm_navigation {

class WmGlobalNavigationAS: public WmGlobalNavigation {
public:
	WmGlobalNavigationAS(std::string name);
	~WmGlobalNavigationAS();

	 void goalCB();
	 void preemptCB();

	 void step();

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<wm_navigation::wm_navigation_alAction> as_;
  std::string action_name_;

  geometry_msgs::PoseStamped goalPose_;

  wm_navigation::wm_navigation_alFeedback feedback_;
  wm_navigation::wm_navigation_alResult result_;


};

} /* namespace wm_navigation */

#endif /* WMGLOBALNAVIGATIONAS_H_ */

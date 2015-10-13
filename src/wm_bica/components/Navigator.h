/*
 * Navigator.h
 *
 *  Created on: 13/10/2015
 *      Author: paco
 */

#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include "wm_bica/Component.h"
#include "wm_bica/Singleton.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <watermellon/wm_navigation_alAction.h>
#include <wm_navigation/Locations.h>
#include <geometry_msgs/PoseStamped.h>

class Navigator: public Component, public Singleton<Navigator> {
public:
	Navigator();
	virtual ~Navigator();

	void step();

	void setGoal(const geometry_msgs::Pose& pose);
	bool finished();

	void doneCb(const actionlib::SimpleClientGoalState& state,
			const watermellon::wm_navigation_alResultConstPtr& result);
	void activeCb();
	void feedbackCb(const watermellon::wm_navigation_alFeedbackConstPtr& feedback);

private:

	ros::NodeHandle n_;

	bool finished_;
	bool new_goal_;
	geometry_msgs::Pose new_pose_;

	actionlib::SimpleActionClient<watermellon::wm_navigation_alAction> ac_;

};

#endif /* NAVIGATOR_H_ */

/*
 * WmGlobalNavigationAS.cpp
 *
 *  Created on: 08/10/2015
 *      Author: paco
 */

#include "wm_navigation/WmGlobalNavigationAS.h"

namespace wm_navigation
{

WmGlobalNavigationAS::WmGlobalNavigationAS(std::string name):
				WmGlobalNavigation(),
				as_(nh_, name, false),
				action_name_(name),
				goalPose_()
{
	//register the goal and feeback callbacks
	as_.registerGoalCallback(boost::bind(&WmGlobalNavigationAS::goalCB, this));
	as_.registerPreemptCallback(boost::bind(&WmGlobalNavigationAS::preemptCB, this));
	as_.start();

	//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
	//		ros::console::notifyLoggerLevelsChanged();

}

WmGlobalNavigationAS::~WmGlobalNavigationAS() {
	// TODO Auto-generated destructor stub
}

void
WmGlobalNavigationAS::goalCB()
{
	goalPose_ = as_.acceptNewGoal()->goal;

	ROS_INFO("New goal to %lf %lf", goalPose_.pose.position.x, goalPose_.pose.position.y);

	feedback_.percent_complete=0.0;

	setGoalPose(goalPose_);
}

void
WmGlobalNavigationAS::preemptCB()
{
	ROS_INFO("%s: Preempted", action_name_.c_str());
	as_.setPreempted();
}

void
WmGlobalNavigationAS::step()
{

	WmGlobalNavigation::step();

	if (!as_.isActive())
		return;

	geometry_msgs::Pose start = getStartingPose();
	geometry_msgs::Pose end = getEndPose();
	geometry_msgs::Pose current = getCurrentPose();

	float startDiffDist, currentDiffDist;
	float startDiffAngle, currentDiffAngle;

	startDiffDist 	= wm_navigation::distance(start, end);
	currentDiffDist	= wm_navigation::distance(current, end);
	startDiffAngle	= fabs(wm_navigation::angleZ(start, end));
	currentDiffAngle= fabs(wm_navigation::angleZ(current, end));

	float totalDiff = startDiffDist + M_PI;
	float currentDiff;
	if(currentDiffDist > 3.0)
		currentDiff = currentDiffDist+M_PI;
	else
		currentDiff = currentDiffDist + currentDiffAngle;

	ROS_DEBUG("Initial Distance to goal [%f]", totalDiff);;
	ROS_DEBUG("Current Distance to goal [%f]", currentDiff);;
	ROS_DEBUG("Current Angle to goal [%f]", currentDiffAngle);;

	ROS_DEBUG("[%f]", feedback_.percent_complete);;
	feedback_.percent_complete = 100.0 * (1.0-currentDiff/totalDiff);

	if(feedback_.percent_complete<0.0) feedback_.percent_complete=0.0;
	if(feedback_.percent_complete>100.0) feedback_.percent_complete=100.0;

	as_.publishFeedback(feedback_);

	if(feedback_.percent_complete >= 99.0)
	{
		ROS_DEBUG("%s: Succeeded", action_name_.c_str());
		// set the action state to succeeded
		result_.finished = true;
		as_.setSucceeded(result_);
	}
}


} /* namespace wm_navigation */

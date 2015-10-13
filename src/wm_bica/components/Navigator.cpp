/*
 * Navigator.cpp
 *
 *  Created on: 13/10/2015
 *      Author: paco
 */

#include "Navigator.h"

Navigator::Navigator():
		ac_("wm_do_global_navigation", true),
		finished_(false),
		new_goal_(false),
		new_pose_()
{
	setFreqTime(LONG_RATE);
}

Navigator::~Navigator()
{

}

bool
Navigator::finished()
{
	return finished_;
}

void
Navigator::doneCb(const actionlib::SimpleClientGoalState& state,
			const watermellon::wm_navigation_alResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	ROS_INFO("Answer: %s", result->finished? "True": "False");

	finished_ = result->finished;

}

void
Navigator::activeCb()
{
	ROS_INFO("Goal just went active");
}

void
Navigator::feedbackCb(const watermellon::wm_navigation_alFeedbackConstPtr& feedback)
{
	ROS_INFO("Got Feedback of length %f", feedback->percent_complete);
}

void
Navigator::setGoal(const geometry_msgs::Pose& pose)
{
	new_pose_ = pose;
	new_goal_ = true;
	finished_ = false;
}

void
Navigator::step()
{
	if(isTime2Run())
	{
		if(ac_.isServerConnected() && new_goal_)
		{
			watermellon::wm_navigation_alGoal goal;
			goal.goal.pose = new_pose_;
			ac_.sendGoal(goal,
						boost::bind(&Navigator::doneCb, this, _1, _2),
						boost::bind(&Navigator::activeCb, this),
						boost::bind(&Navigator::feedbackCb, this, _1));

			new_goal_ = false;
		}
	}

}

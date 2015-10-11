/*
 * mapper_node.cpp
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <wm_navigation/wm_navigation_alAction.h>

using namespace wm_navigation;


void doneCb(const actionlib::SimpleClientGoalState& state,
		const wm_navigation::wm_navigation_alResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	ROS_INFO("Answer: %s", result->finished? "True": "False");
	ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
	ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const wm_navigation_alFeedbackConstPtr& feedback)
{
	ROS_INFO("Got Feedback of length %f", feedback->percent_complete);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, std::string("test_wm_global_navigation"));
	ros::NodeHandle n;

	actionlib::SimpleActionClient<wm_navigation::wm_navigation_alAction> ac("wm_do_global_navigation", true);

	ROS_INFO("Waiting for action server to start.");

	ac.waitForServer();

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	wm_navigation::wm_navigation_alGoal goal;

	goal.goal.pose.position.x = 6.0;
	goal.goal.pose.position.y = 3.0;
	goal.goal.pose.position.z = 0.0;

	goal.goal.pose.orientation.x = 0.0;
	goal.goal.pose.orientation.y = 0.0;
	goal.goal.pose.orientation.z = 0.0;
	goal.goal.pose.orientation.w = 1.0;

	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

	ros::Rate loop_rate(5);


	try{
		while ( ros::ok() && ac.getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
	}catch(std::runtime_error& e){
		ROS_ERROR("wm_map_server exception: %s", e.what());
		return -1;
	}

	return 0;

}

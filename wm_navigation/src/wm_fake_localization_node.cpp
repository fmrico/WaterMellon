/*
 * mapper_node.cpp
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */


#include "ros/ros.h"
#include <string>
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <iostream>

#include <math.h>

namespace wm_localization {

class WmFakeLocalization {
public:
	WmFakeLocalization() {
		posecov_pub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/wm_pose_cov", 1, true);
		pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/wm_pose", 1, true);

		submodel = m_nh.subscribe("/gazebo/model_states", 1000, &WmFakeLocalization::groundtruthCB, this);

	}

	void step() {

		if(posecov_pub.getNumSubscribers() > 0)
		{
			geometry_msgs::PoseWithCovarianceStamped pose2send;

			pose2send.header.frame_id = "/map";
			pose2send.header.stamp = ros::Time::now();
			pose2send.pose = posecv;

			posecov_pub.publish(pose2send);
		}

		if(pose_pub.getNumSubscribers() > 0)
			{
				geometry_msgs::PoseStamped pose2send;

				pose2send.header.frame_id = "/map";
				pose2send.header.stamp = ros::Time::now();
				pose2send.pose = posecv.pose;

				pose_pub.publish(pose2send);
			}

	}

	void groundtruthCB(const gazebo_msgs::ModelStates::ConstPtr& states)
	{
		ROS_DEBUG("groundtruthCB Called");
		geometry_msgs::Pose gt;
		for(int i=0; i< states->name.size(); i++)
		{
			//ROS_DEBUG("[%s]", states->name[i].c_str());
			if(states->name[i].find("robot")!=std::string::npos)
				gt = states->pose[i];
		}

		//////////////////////////////////////////////////////////
		tf::Transform G2M;
		geometry_msgs::Transform gt_res;

		G2M.setOrigin(tf::Vector3(0.0f, 0.0f, 0.0f));
		tf::Quaternion q;
		q.setEuler(0.0, 0.0, M_PI);
		G2M.setRotation(q);

		tf::Transform tf_gt;
		tf::poseMsgToTF(gt, tf_gt);


		tf_gt = G2M*tf_gt ;
		tf::transformTFToMsg(tf_gt, gt_res);

		gt.position.x = gt_res.translation.x;
		gt.position.y = gt_res.translation.y;
		gt.position.z = gt_res.translation.z;
		gt.orientation = gt_res.rotation;
		///////////////////////////////////////////////////////////
		posecv.pose = gt;
		posecv.covariance[0] = 0.01;
		posecv.covariance[1 * 6 + 1] =0.01;
		posecv.covariance[5 * 6 + 5] = 0.01;

	}

private:
	ros::Publisher posecov_pub;
	ros::Publisher pose_pub;
	ros::Subscriber submodel;
	ros::NodeHandle m_nh;

	geometry_msgs::PoseWithCovariance posecv;
};
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, std::string("wm_localization"));
	ros::NodeHandle n;

  ros::Rate loop_rate(5);

  wm_localization::WmFakeLocalization wmfakelocalization;

  try{
	   while ( ros::ok())
	   {

       wmfakelocalization.step();

       ros::spinOnce();
       loop_rate.sleep();
     }
	}catch(std::runtime_error& e){
		ROS_ERROR("wm_map_server exception: %s", e.what());
		return -1;
	}

	return 0;

}

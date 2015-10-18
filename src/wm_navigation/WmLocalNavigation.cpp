/*
 * mapper.cpp
 *
 *  Created on: 22/08/2015
 *      Author: paco
 */

#include "wm_navigation/WmLocalNavigation.h"

namespace wm_navigation{


WmLocalNavigation::WmLocalNavigation(ros::NodeHandle private_nh)
: nh_(),
  robot_radious_(0.177),
  max_w_(0.15),
  max_v_(0.2),
  collision_area_(0.5),
  worldFrameId_("/map"), baseFrameId_("base_footprint"),
  tfListener_(),
  atractive_vector_(new geometry_msgs::TwistStamped),
  resultant_vector_(new geometry_msgs::TwistStamped),
  repulsive_vector_(new geometry_msgs::TwistStamped),
  scan_bf(),
  goal_(new watermellon::GNavGoalStamped),
  state_(FINISHED)
{
	private_nh.param("frame_id", worldFrameId_, worldFrameId_);
	private_nh.param("base_frame_id", baseFrameId_, baseFrameId_);
	private_nh.param("robot_radious", robot_radious_, robot_radious_);
	private_nh.param("collision_area", collision_area_, collision_area_);


	scanSub_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "scan", 5);
	tfScanSub_ = new tf::MessageFilter<sensor_msgs::LaserScan> (*scanSub_, tfListener_, baseFrameId_, 5);
	tfScanSub_->registerCallback(boost::bind(&WmLocalNavigation::scanCallback, this, _1));

	goal_sub_ = nh_.subscribe<watermellon::GNavGoalStamped>("/goal_vector", 1000, &WmLocalNavigation::gvectorCallback, this);

	repulsive_vector_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/repulsive_vector", 1000);
	resultant_vector_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/resultant_vector", 1000);
	atractive_vector_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/atractive_vector", 1000);

	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	atractive_vector_->header.stamp.init();
}

WmLocalNavigation::~WmLocalNavigation() {

}



void
WmLocalNavigation::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	float o_t_min, o_t_max, o_t_inc;

	o_t_min = scan_in->angle_min;
	o_t_max = scan_in->angle_max;
	o_t_inc = scan_in->angle_increment;

	int num_points = (int)2.0*o_t_max/o_t_inc;

	tf::Stamped<tf::Point> scan_sensor[num_points];
	scan_bf.resize(num_points);

	float rx=0.0, ry=0.0;
	int c=0;

	for(int i=0; i<num_points; i++)
	{
		float theta = o_t_min+i*o_t_inc;
		float r = scan_in->ranges[i];

		scan_sensor[i].setX(r*cos(theta));
		scan_sensor[i].setY(r*sin(theta));
		scan_sensor[i].setZ(0.0);
		scan_sensor[i].setW(1.0);
		scan_sensor[i].stamp_ = scan_in->header.stamp;
		scan_sensor[i].frame_id_ = scan_in->header.frame_id;

		tfListener_.transformPoint(baseFrameId_, scan_sensor[i], scan_bf[i]);

		if(scan_in->ranges[i]<collision_area_)
		{
			rx = rx + -scan_bf[i].getX();// -(collision_area_-scan_bf[i].getX());
			ry = ry + -scan_bf[i].getY();// -(collision_area_-scan_bf[i].getY());
			c++;
		}
	}


	if(c>0)
	{
		rx = rx/c;
		ry = ry/c;
	}


	repulsive_vector_->header.frame_id = baseFrameId_;
	repulsive_vector_->header.stamp = scan_in->header.stamp;

	float m = sqrt(rx*rx+ry*ry);
	repulsive_vector_->twist.linear.x = m*m;
	repulsive_vector_->twist.linear.y = 0.0;
	repulsive_vector_->twist.linear.z = 0.0;
	repulsive_vector_->twist.angular.x = 0.0;
	repulsive_vector_->twist.angular.y = 0.0;
	repulsive_vector_->twist.angular.z = atan2(ry, rx);

}

void
WmLocalNavigation::gvectorCallback(const watermellon::GNavGoalStamped::ConstPtr& gvector_in)
{

	ROS_DEBUG("Global Vector received");
	*goal_ = *gvector_in;

}

void
WmLocalNavigation::addVectors(geometry_msgs::Twist& v1, const geometry_msgs::Twist& v2)
{


	//ROS_INFO("(%lf, %lf) += (%lf, %lf)", v1->twist.linear.x, v1->twist.angular.z, v2->twist.linear.x, v2->twist.angular.z);

	if(fabs(v2.linear.x)>0.00001)
	{
		//ROS_INFO("CASO 1");
		float x1, x2, y1, y2;

		x1 = v1.linear.x * cos(v1.angular.z);
		y1 = v1.linear.x * sin(v1.angular.z);
		x2 = v2.linear.x * cos(v2.angular.z);
		y2 = v2.linear.x * sin(v2.angular.z);

		float xr, yr;
		xr = x1+x2;
		yr = y1+y2;


		v1.linear.x = sqrt (xr*xr+yr*yr);
		v1.angular.z = atan2(yr, xr);
	}else
	{
		//ROS_INFO("CASO 2");
		//v1->twist.linear.x = v2->twist.linear.x;
		v1.angular.z = normalizePi(v1.angular.z + v2.angular.z);
	}
}

void
WmLocalNavigation::checkSafeVel(geometry_msgs::Twist& v)
{
	if(scan_bf.empty() || (scan_bf[0].stamp_-ros::Time::now()>ros::Duration(1.0)))
	{
		ROS_DEBUG("NO Scan Data");
		return;
	}else
	{
		ROS_DEBUG("Scan Data available");

		if(v.linear.x > 0.0)
		{
			std::vector<tf::Stamped<tf::Point> >::iterator it;

			for(it=scan_bf.begin(); it<scan_bf.end(); ++it)
			{
				float dist=sqrt(it->x()*it->x()+it->y()*it->y());

				if((fabs(it->y())<robot_radious_) && (dist>robot_radious_) && (dist<robot_radious_*1.5))
				{
					ROS_WARN("Collision detected by laser at %lf %lf", it->x(), it->y());

					v.linear.x = 0.0;
					break;
				}
			}
		}
		return;
	}
}




void
WmLocalNavigation::step()
{

	ros::WallTime startTime = ros::WallTime::now();
	double total_elapsed = (ros::WallTime::now() - startTime).toSec();

	resultant_vector_->header.frame_id = baseFrameId_;
	resultant_vector_->header.stamp = ros::Time::now();
	resultant_vector_->twist.linear.x = 0.0;
	resultant_vector_->twist.linear.y = 0.0;
	resultant_vector_->twist.linear.z = 0.0;
	resultant_vector_->twist.angular.x = 0.0;
	resultant_vector_->twist.angular.y = 0.0;
	resultant_vector_->twist.angular.z = 0.0;

	geometry_msgs::Twist vel;

	atractive_vector_->twist = goal_->gradient;

	switch(state_){

	case FAR:

		ROS_DEBUG("FAR");
		if(goal_->distance_goal<0.7)
			state_=NEAR;



		if((ros::Time::now() - goal_->header.stamp) < ros::Duration(1.0))
			addVectors(resultant_vector_->twist, atractive_vector_->twist);
		if((ros::Time::now() - repulsive_vector_->header.stamp ) < ros::Duration(1.0))
			addVectors(resultant_vector_->twist, repulsive_vector_->twist);

		if(fabs(resultant_vector_->twist.angular.z) > M_PI/4.0)
		{
			vel.linear.x = 0.0;
			vel.angular.z = resultant_vector_->twist.angular.z;
		}else
		{
			vel.linear.x = resultant_vector_->twist.linear.x;
			vel.angular.z = resultant_vector_->twist.angular.z;
		}

		ROS_DEBUG("REPULSIVE = (%lf, %lf)", repulsive_vector_->twist.linear.x, repulsive_vector_->twist.angular.z);
		ROS_DEBUG("ATRACTIVE = (%lf, %lf)", atractive_vector_->twist.linear.x, atractive_vector_->twist.angular.z);
		ROS_DEBUG("RESULTANT = (%lf, %lf)", resultant_vector_->twist.linear.x, resultant_vector_->twist.angular.z);


		break;
	case NEAR:
		ROS_DEBUG("NEAR");
		if(goal_->distance_goal<0.15)
			state_=TURNING;
		else if(goal_->distance_goal>0.8)
			state_=FAR;

		if((ros::Time::now() - goal_->header.stamp) < ros::Duration(1.0))
			addVectors(resultant_vector_->twist, atractive_vector_->twist);


		vel.linear.x = resultant_vector_->twist.linear.x;
		vel.angular.z = resultant_vector_->twist.angular.z;

		if(fabs(resultant_vector_->twist.angular.z) > M_PI/2.0)
		{
			vel.linear.x = 0.0;
			vel.angular.z = resultant_vector_->twist.angular.z;
		}else
		{
			vel.linear.x = resultant_vector_->twist.linear.x;
			vel.angular.z = resultant_vector_->twist.angular.z;
		}

		break;
	case TURNING:
		ROS_DEBUG("TURNING");
		if(goal_->distance_goal>0.3)
			state_=NEAR;
		else if(fabs(goal_->final.angular.z) < 0.17)
			state_=FINISHED;

		vel.linear.x = 0.0;
		vel.angular.z = goal_->final.angular.z;

		break;
	case FINISHED:
		ROS_DEBUG("FINISHED");
		if(goal_->distance_goal>0.8)
			state_=FAR;
		else if(goal_->distance_goal>0.3)
			state_=NEAR;
		else if(fabs(goal_->final.angular.z) > 0.25)
			state_=TURNING;

		vel.linear.x = 0.0;
		vel.angular.z = 0.0;
		break;
	}


	if(vel.angular.z>max_w_)
		vel.angular.z = max_w_;
	if(vel.angular.z<-max_w_)
		vel.angular.z = -max_w_;
	if(vel.linear.x > max_v_)
		vel.linear.x = max_v_;
	if(vel.linear.x < -max_v_)
		vel.linear.x = -max_v_;

	if(goal_->localization_quality<0.9)
	{
		vel.angular.z = vel.angular.z/3.0;
		vel.linear.x = vel.linear.x/3.0;
	}

	if((ros::Time::now() - repulsive_vector_->header.stamp ) < ros::Duration(1.0))
		checkSafeVel(vel);

	//vel.linear.x = 0.0;
	//vel.angular.z = 0.0;

	vel_pub_.publish(vel);

	ROS_DEBUG("LocalNavigation done (%f sec)", total_elapsed);

	publish_all();


}

void
WmLocalNavigation::publish_all()
{

	publish_resultant_vector();
	publish_repulsive_vector();
	publish_atractive_vector();
}

void
WmLocalNavigation::publish_repulsive_vector()
{
	if(repulsive_vector_pub_.getNumSubscribers() == 0) return;

	repulsive_vector_pub_.publish(repulsive_vector_);

}

void
WmLocalNavigation::publish_atractive_vector()
{
	if(atractive_vector_pub_.getNumSubscribers() == 0) return;

	atractive_vector_pub_.publish(repulsive_vector_);

}

void
WmLocalNavigation::publish_resultant_vector()
{
	if(resultant_vector_pub_.getNumSubscribers() == 0) return;

	resultant_vector_pub_.publish(resultant_vector_);

}

}

/*
 * main.cpp
 *
 *  Created on: 13 May 2013
 *      Author: Murilo F. M.
 *      Email: muhrix@gmail.com
 *
 */

#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "tf/transform_broadcaster.h"

#include <boost/bind.hpp>

void BroadcastCameraLinkTF(tf::TransformBroadcaster &br,
		const std_msgs::Float64ConstPtr& tilt) {
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0.15, 0.0, 0.9) );
	transform.setRotation( tf::createQuaternionFromRPY(0.0, tilt->data*M_PI/180.0, 0.0) );
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_link"));
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "camera_link_broadcaster");
	ros::NodeHandle nh;
	tf::TransformBroadcaster br;
	ros::Subscriber sub = nh.subscribe<std_msgs::Float64>("/cur_tilt_angle", 1,
			boost::bind(&BroadcastCameraLinkTF, boost::ref(br), _1));
	ros::spin();

	std::cout << std::endl;
	return 0;
}



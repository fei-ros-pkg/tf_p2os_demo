/*
 * TFListener.cpp
 *
 *  Created on: 4 Dec 2013
 *      Author: Murilo F. M.
 *      Email: muhrix@gmail.com
 *
 */

#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_listener.h"

#include <boost/bind.hpp>

void RobotPoseCallback(ros::Publisher &pub, tf::TransformListener &listener,
		tf::StampedTransform &transform,
		const geometry_msgs::PoseWithCovarianceConstPtr& pose) {

	try {
		listener.lookupTransform("/odom", "/base_link",
    		ros::Time(0), transform);

		geometry_msgs::Twist velCmd;
		velCmd.linear.x = 0.0;
		velCmd.angular.z = 0.0;

		pub.publish(velCmd);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
	}
}

int main(int argc, char* argv[]) {
	std::cout << "Starting TF listener..." << std::endl;

	ros::init(argc, argv, "TFListener");

	ros::NodeHandle myNode;

	ros::Publisher velPub = myNode.advertise<geometry_msgs::Twist>(
			"/cmd_vel", 1);

	tf::TransformListener listener;
	tf::StampedTransform transform;

	ros::SubscribeOptions subOpts;
	subOpts.topic = "/odom";
	subOpts.queue_size = 1;

	ros::Subscriber poseSub = myNode.subscribe<geometry_msgs::PoseWithCovariance>(
			"/odom", 1,
			boost::bind(&RobotPoseCallback, boost::ref(velPub),
			boost::ref(listener), boost::ref(transform), _1));

	ros::Rate loop_rate(10);

	ros::spin();

	std::cout << "\nShutting down TF Listener." << std::endl;

	return 0;
}

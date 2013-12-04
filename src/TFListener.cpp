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

//void robotPoseCallback(ros::NodeHandle &node_handle,
//		const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {

void RobotPoseCallback(ros::Publisher &pub, SpiralBehaviour &sb, tf::TransformListener &listener,
		tf::StampedTransform &transform,
		const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {

	try {
		listener.lookupTransform("/odom", "/base_link",
    		ros::Time(0), transform);

		geometry_msgs::Twist spiralCmd;
		spiralCmd = sb.CalculateVels(transform);

		pub.publish(spiralCmd);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
	}
}

int main(int argc, char* argv[]) {
	std::cout << "Starting TF listener..." << std::endl;

	ros::init(argc, argv, "TFListener");

	ros::NodeHandle myNode;

	ros::Publisher spiralPub = myNode.advertise<geometry_msgs::Twist>(
			"/cmd_vel", 1);

	tf::TransformListener listener;
	tf::StampedTransform transform;

	ros::SubscribeOptions subOpts;
	subOpts.topic = "/odom";
	subOpts.queue_size = 1;


	ros::Subscriber spiral_sub = myNode.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
			"/robot_pose_ekf/odom_combined", 1,
			boost::bind(&RobotPoseCallback, boost::ref(spiralPub), boost::ref(sb),
			boost::ref(listener), boost::ref(transform), _1));

	ros::Rate loop_rate(10);

	ros::spin();

	std::cout << "\nShutting down TF Listener." << std::endl;

	return 0;
}

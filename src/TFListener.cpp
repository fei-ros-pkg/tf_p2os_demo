/*
 * TFListener.cpp
 *
 *  Created on: 4 Dec 2013
 *      Author: Murilo F. M.
 *      Email: muhrix@gmail.com
 *
 */

#include <iostream>
#include <exception>
#include <cmath>
// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"

#include <boost/bind.hpp>

void RobotPoseCallback(ros::Publisher &pub, tf::TransformListener &listener,
		tf::StampedTransform &transform,
		const nav_msgs::OdometryConstPtr& odom) {

	geometry_msgs::PoseStamped robotPose;
	geometry_msgs::PoseStamped kinectPose;
	geometry_msgs::Twist velCmd;
	velCmd.linear.x = 0.0;
	velCmd.angular.z = 0.0;
	tf::Pose tfPose;

	try {
		// Print robot pose (from odometry)
		robotPose.header = odom->header;
		robotPose.pose = odom->pose.pose;

		tf::poseMsgToTF(robotPose.pose, tfPose);

		ROS_INFO("Robot.x: %.2f", robotPose.pose.position.x);
		ROS_INFO("Robot.y: %.2f", robotPose.pose.position.y);
		ROS_INFO("Robot.z: %.2f", robotPose.pose.position.z);
		ROS_INFO("Robot.yaw: %.2f", tf::getYaw(tfPose.getRotation()));
		ROS_INFO("Robot.yaw: %.2f", tf::getYaw(robotPose.pose.orientation));

		listener.transformPose("/camera_depth_optical_frame",
    		robotPose, kinectPose);

		tf::poseMsgToTF(kinectPose.pose, tfPose);

		ROS_INFO("Kinect.x: %.2f", kinectPose.pose.position.x);
		ROS_INFO("Kinect.y: %.2f", kinectPose.pose.position.y);
		ROS_INFO("Kinect.z: %.2f", kinectPose.pose.position.z);
		ROS_INFO("Kinect.yaw: %.2f", tf::getYaw(tfPose.getRotation()));
		ROS_INFO("Kinect.yaw: %.2f", tf::getYaw(kinectPose.pose.orientation));

		pub.publish(velCmd);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
	}
}

int main(int argc, char* argv[]) {
	std::cout << "Starting TF listener..." << std::endl;

	ros::init(argc, argv, "tf_listener_demo");

	ros::NodeHandle myNode;

	ros::Publisher velPub = myNode.advertise<geometry_msgs::Twist>(
			"/cmd_vel", 1);

	tf::TransformListener listener;
	tf::StampedTransform transform;

	ros::SubscribeOptions subOpts;
	subOpts.topic = "/pose";
	subOpts.queue_size = 1;

	ros::Subscriber poseSub = myNode.subscribe<nav_msgs::Odometry>(
			"/pose", 1,
			boost::bind(&RobotPoseCallback, boost::ref(velPub),
			boost::ref(listener), boost::ref(transform), _1));

	ros::Rate loop_rate(10);

	ros::spin();

	std::cout << "\nShutting down TF listener." << std::endl;

	return 0;
}

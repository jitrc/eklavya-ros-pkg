#include <iostream>
#include <stdio.h>

#include <ros/ros.h>

#include <eklavya_odometry/odometry.h>
#include <eklavya_encoder/encoder.h>

using namespace std;

ros::Publisher odometry_publisher;
nav_msgs::Odometry odometry_message;

int sequence_id = 0;
ros::Time last_time, current_time;
ros::Duration duration;
double pose_covariance_matrix[36] = { 1, 0, 0, 0, 0, 0,
									   0, 1, 0, 0, 0, 0,
							  		   0, 0, 1, 0, 0, 0,
									   0, 0, 0, 1, 0, 0,
									   0, 0, 0, 0, 1, 0,
							   		   0, 0, 0, 0, 0, 1};
double twist_covariance_matrix[36] = { 1, 0, 0, 0, 0, 0,
												  0, 1, 0, 0, 0, 0,
												  0, 0, 1, 0, 0, 0,
												  0, 0, 0, 1, 0, 0,
												  0, 0, 0, 0, 1, 0,
												  0, 0, 0, 0, 0, 1};

void encoderCallback(const eklavya_encoder::Encoder_Data::ConstPtr& msg) {
	ROS_INFO("Encoder data received...");
	
	if (sequence_id == 0) {
		last_time = ros::Time::now();
	}
	
	current_time = ros::Time::now();
	duration = current_time - last_time;
	
	odometry_message.header.seq = sequence_id++;
	odometry_message.header.stamp = current_time;
	odometry_message.header.frame_id = "/map";
	
	odometry_message.child_frame_id = "/base_link";
	
	odometry_message.pose.pose.position.x = 0;
	odometry_message.pose.pose.position.y = 0;
	odometry_message.pose.pose.position.z = 0;
	odometry_message.pose.pose.orientation.x = 0;
	odometry_message.pose.pose.orientation.y = 0;
	odometry_message.pose.pose.orientation.z = 0;
	odometry_message.pose.pose.orientation.w = 0;
	for (int i = 0; i < 36; i++) {
		odometry_message.pose.covariance[i] = pose_covariance_matrix[i];
	}
	
	odometry_message.twist.twist.linear.x = 0;
	odometry_message.twist.twist.linear.y = 0;
	odometry_message.twist.twist.linear.z = 0;
	odometry_message.twist.twist.angular.x = 0;
	odometry_message.twist.twist.angular.y = 0;
	odometry_message.twist.twist.angular.z = 0;
	for (int i = 0; i < 36; i++) {
		odometry_message.twist.covariance[i] = twist_covariance_matrix[i];
	}
	
	odometry_publisher.publish(odometry_message);
	
}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "odometry");
	
    ros::NodeHandle n;
    odometry_publisher = n.advertise<nav_msgs::Odometry>("odometry", 30, true);
    ros::Subscriber encoder_subscriber = n.subscribe("encoder", 1, encoderCallback);
    
    printf("Odometry node initialized...\n");
    
    ros::spin();
	
	return 0;
}

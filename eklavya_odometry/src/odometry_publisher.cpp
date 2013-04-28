#include <iostream>
#include <stdio.h>

#include <ros/ros.h>

#include <eklavya_odometry/odometry.h>
#include <eklavya_encoder/encoder.h>

using namespace std;

ros::Publisher odometry_publisher;
nav_msgs::Odometry odometry_message;

odometry_space::OdometryFactory odometry_factory;

void encoderCallback(const eklavya_encoder::Encoder_Data::ConstPtr& msg) {
	
	ROS_INFO("Encoder data received...");
	
	odometry_message = odometry_factory.getOdometryData(msg);
	
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

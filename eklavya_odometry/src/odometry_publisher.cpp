#include <iostream>
#include <stdio.h>

#include <ros/ros.h>

#include <eklavya_odometry/odometry.h>

using namespace std;

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "odometry");
	
    ros::NodeHandle n;
    ros::Publisher odometry_publisher = n.advertise<nav_msgs::Odometry>("odometry", 30, true);
    
    nav_msgs::Odometry odometry_data;
    
    printf("Odometry node initialized...\n");
    
    int i = 0;
    while (ros::ok()) {
		printf("Odometry running...\n");
		usleep(999999);
	}
	
	return 0;
}

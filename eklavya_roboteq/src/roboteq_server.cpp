#include <iostream>
#include <stdio.h>
#include <string.h>

#include "ros/ros.h"
#include "eklavya_roboteq/SetSpeed.h"

#include "eklavya_roboteq/RoboteqDevice.h"
#include "eklavya_roboteq/ErrorCodes.h"
#include "eklavya_roboteq/Constants.h"

using namespace std;

RoboteqDevice device;
int status = 0;

bool setSpeed(eklavya_roboteq::SetSpeed::Request  &req, eklavya_roboteq::SetSpeed::Response &res) {
    ROS_INFO("request: Left motor speed = %ld, Right motor speed = %ld", (long int)req.left_speed, (long int)req.right_speed);
    
	if((status = device.SetCommand(_GO, 1, 1)) != RQ_SUCCESS)
		cout<<"failed --> "<<status<<endl;
	else
		cout<<"succeeded."<<endl;
    
    res.code = 0;
    ROS_INFO("sending back response: [%ld]", (long int)res.code);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller_server");
    ros::NodeHandle n;
    
    string response = "";
	RoboteqDevice device;
	status = device.Connect("/dev/ttyS0");

	if(status != RQ_SUCCESS)
	{
		cout<<"Error connecting to device: "<<status<<"."<<endl;
		return 1;
	}
  
    ros::ServiceServer service = n.advertiseService("motor_controller", add);
    ROS_INFO("Ready to control motors.");
    
    ros::spin();
	
	device.Disconnect();
  
    return 0;
}

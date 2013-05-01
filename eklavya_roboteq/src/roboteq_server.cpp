#include <iostream>
#include <stdio.h>
#include <string.h>

#include "ros/ros.h"
#include "../srv_gen/cpp/include/eklavya_roboteq/SetSpeed.h"
#include "../srv_gen/cpp/include/eklavya_roboteq/GetSpeed.h"

#include "eklavya_roboteq/RoboteqDevice.h"
#include "eklavya_roboteq/ErrorCodes.h"
#include "eklavya_roboteq/Constants.h"

using namespace std;

RoboteqDevice device;
int status = 0;

bool setSpeed(eklavya_roboteq::SetSpeed::Request  &req, eklavya_roboteq::SetSpeed::Response &res) {
    ROS_INFO("request: Left motor speed = %ld, Right motor speed = %ld", (long int)req.left_speed, (long int)req.right_speed);
    
	if((status = device.SetCommand(_GO, 1, req.left_speed)) != RQ_SUCCESS)
		cout<<"failed --> "<<status<<endl;
	else
		cout<<"succeeded."<<endl;
		
	usleep(100);
		
	if((status = device.SetCommand(_GO, 2, req.right_speed)) != RQ_SUCCESS)
		cout<<"failed --> "<<status<<endl;
	else
		cout<<"succeeded."<<endl;
    
    res.code = 0;
    ROS_INFO("sending back response: [%ld]", (long int)res.code);
    return true;
}

bool getSpeed(eklavya_roboteq::GetSpeed::Request &req, eklavya_roboteq::GetSpeed::Response &res) {
	ROS_INFO("request: Encoder speed data.");
	
	int left_speed = 0, right_speed = 0;
	
	if((status = device.GetValue(_ABSPEED, 1, left_speed)) != RQ_SUCCESS)
		cout<<"failed --> "<<status<<endl;
	else
		cout<<"succeeded."<<endl;
		
	usleep(100);
		
	if((status = device.GetValue(_ABSPEED, 2, right_speed)) != RQ_SUCCESS)
		cout<<"failed --> "<<status<<endl;
	else
		cout<<"succeeded."<<endl;
		
	res.left_speed = left_speed;
	res.right_speed = right_speed;
	
	return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller_server");
    ros::NodeHandle n;
    
    string response = "";
	status = device.Connect("/dev/serial/by-id/usb-Roboteq_Motor_Controller_498954A73235-if00");
	//status = device.Connect("/dev/ttyACM0");

	if(status != RQ_SUCCESS)
	{
		cout<<"Error connecting to device: "<<status<<"."<<endl;
		return 1;
	}
  
    ros::ServiceServer service1 = n.advertiseService("motor_controller", setSpeed);
    ros::ServiceServer service2 = n.advertiseService("motor_speed", getSpeed);
    ROS_INFO("Ready to control motors.");
    
    ros::spin();
	
	device.Disconnect();
  
    return 0;
}

#include <eklavya_odometry/odometry.h>

namespace odometry_space {
	
	OdometryFactory::OdometryFactory() {
		sequence_id = 0;
		for (int i = 0; i < 36; i++) {
			if (i/6 == i%6) {
				pose_covariance_matrix[i] = 1;
				twist_covariance_matrix[i] = 1;
			} else {
				pose_covariance_matrix[i] = 0;
				twist_covariance_matrix[i] = 0;
			}
		}
		wheel_separation = 0.55;
	}

	nav_msgs::Odometry OdometryFactory::getOdometryData(const eklavya_encoder::Encoder_Data::ConstPtr& msg) {
		nav_msgs::Odometry odometryData;
		
		if (sequence_id == 0) {
			last_time = ros::Time::now();
		}
	
		current_time = ros::Time::now();
		duration = current_time - last_time;
		
		odometryData.header.seq = sequence_id++;
		odometryData.header.stamp = current_time;
		odometryData.header.frame_id = "/map";
	
		odometryData.child_frame_id = "/base_link";
		
		odometryData.twist.twist.linear.x = (msg->left_count + msg->right_count)/2;
		odometryData.twist.twist.linear.y = 0;  //Fixed
		odometryData.twist.twist.linear.z = 0;  //Fixed
		odometryData.twist.twist.angular.x = 0; //Fixed
		odometryData.twist.twist.angular.y = 0; //Fixed
		odometryData.twist.twist.angular.z = (msg->left_count - msg->right_count)/wheel_separation;
		for (int i = 0; i < 36; i++) {
			odometryData.twist.covariance[i] = twist_covariance_matrix[i];
		}
	
		odometryData.pose.pose.position.x = previous.pose.pose.position.x + odometryData.twist.twist.linear.x * cos(0);	//Angle conversion
		odometryData.pose.pose.position.y = previous.pose.pose.position.y + odometryData.twist.twist.linear.y * sin(0);	//Angle conversion
		odometryData.pose.pose.position.z = 0; //Fixed
		odometryData.pose.pose.orientation.x = 0;	//Angle conversion
		odometryData.pose.pose.orientation.y = 0;	//Angle conversion
		odometryData.pose.pose.orientation.z = 0;	//Angle conversion
		odometryData.pose.pose.orientation.w = 0;	//Angle conversion
		for (int i = 0; i < 36; i++) {
			odometryData.pose.covariance[i] = pose_covariance_matrix[i];
		}
	
		return odometryData;
	}
	
}

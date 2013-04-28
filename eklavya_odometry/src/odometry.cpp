#include <eklavya_odometry/odometry.h>
#include <tf/tf.h>

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
		double current_yaw;
		
		if (sequence_id == 0) {
			last_time = ros::Time::now();
		}
	
		current_time = ros::Time::now();
		duration = current_time - last_time;
		
		//Header
		odometryData.header.seq = sequence_id++;
		odometryData.header.stamp = current_time;
		odometryData.header.frame_id = "/map";
	
		//Child frame
		odometryData.child_frame_id = "/base_link";
		
		//Twist
		odometryData.twist.twist.linear.x = (msg->left_count + msg->right_count)/2;
		odometryData.twist.twist.linear.y = 0;  //Fixed
		odometryData.twist.twist.linear.z = 0;  //Fixed
		
		odometryData.twist.twist.angular.x = 0; //Fixed
		odometryData.twist.twist.angular.y = 0; //Fixed
		odometryData.twist.twist.angular.z = (msg->left_count - msg->right_count)/wheel_separation;
		
		for (int i = 0; i < 36; i++) {
			odometryData.twist.covariance[i] = twist_covariance_matrix[i];
		}
		
		//Orientation conversions and computations
		tf::quaternionMsgToTF(previous.pose.pose.orientation, q);
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
		ROS_DEBUG("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);
		current_yaw = yaw + odometryData.twist.twist.angular.z * duration.toSec();
	
		//Pose
		odometryData.pose.pose.position.x = previous.pose.pose.position.x + odometryData.twist.twist.linear.x * cos(current_yaw) * duration.toSec();	//Angle conversion
		odometryData.pose.pose.position.y = previous.pose.pose.position.y + odometryData.twist.twist.linear.y * sin(current_yaw) * duration.toSec();	//Angle conversion
		odometryData.pose.pose.position.z = 0; //Fixed
		
		q = tf::createQuaternionFromYaw(current_yaw);
		tf::quaternionTFToMsg(q, odometryData.pose.pose.orientation);
				
		for (int i = 0; i < 36; i++) {
			odometryData.pose.covariance[i] = pose_covariance_matrix[i];
		}
		
		previous = odometryData;
	
		return odometryData;
	}
	
}

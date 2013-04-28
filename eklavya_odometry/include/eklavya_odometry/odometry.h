#ifndef _ODOMETRY_H
#define _ODOMETRY_H

#include <nav_msgs/Odometry.h>
#include <eklavya_encoder/encoder.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace odometry_space {
	
	class OdometryFactory {
		
		private:
		int sequence_id;
		double pose_covariance_matrix[36];
		double twist_covariance_matrix[36];
		
		geometry_msgs::TransformStamped transform_stamped;
		tf::TransformBroadcaster odom_broadcaster;
		
		ros::Time last_time, current_time;
		ros::Duration duration;
	
		geometry_msgs::Quaternion quaternion;
		
		double wheel_separation;

		double position_x;
		double position_y;
		double yaw;

		double velocity_x;
		double yaw_rate;
		
		public:
		OdometryFactory();
		void updateOdometryData(const eklavya_encoder::Encoder_Data::ConstPtr&);
		nav_msgs::Odometry getOdometryData();
		void encoderCallback(nav_msgs::Odometry::ConstPtr& msg);
		
	};
	
}

#endif

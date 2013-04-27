#ifndef _ODOMETRY_H
#define _ODOMETRY_H

#include <nav_msgs/Odometry.h>
#include <eklavya_encoder/encoder.h>

namespace odometry_space {
	
	class OdometryFactory {
		
		private:
		int sequence_id;
		ros::Time last_time, current_time;
		ros::Duration duration;
		double pose_covariance_matrix[36];
		double twist_covariance_matrix[36];
		
		public:
		OdometryFactory();
		nav_msgs::Odometry getOdometryData(const eklavya_encoder::Encoder_Data::ConstPtr&);
		void encoderCallback(nav_msgs::Odometry::ConstPtr& msg);
		
	};
	
}

#endif

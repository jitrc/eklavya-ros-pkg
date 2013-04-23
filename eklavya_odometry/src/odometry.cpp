#include <eklavya_odometry/odometry.h>

namespace odometry_space {
	
	nav_msgs::Odometry OdometryFactory::getOdometryData(encoder_space::EncoderData) {
		nav_msgs::Odometry odometryData;
		return odometryData;
	}
	
}

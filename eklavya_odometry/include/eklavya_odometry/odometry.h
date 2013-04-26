#ifndef _ODOMETRY_H
#define _ODOMETRY_H

#include <nav_msgs/Odometry.h>
#include <eklavya_encoder/encoder.h>

namespace odometry_space {
	
	class OdometryFactory {
		
		private:
		time_t time;
		
		public:
		nav_msgs::Odometry getOdometryData(encoder_space::EncoderData);
		void encoderCallback(nav_msgs::Odometry::ConstPtr& msg);
		
	};
	
}

#endif

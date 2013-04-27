#include <ros/ros.h>

#include <stdlib.h>

#include <eklavya_encoder/encoder.h>

namespace encoder_space {
	
	int leftSpeed = 0, rightSpeed = 0;
	
	Encoder::Encoder(int argc, char **argv) {
		srand (time(NULL));
	}
	
	EncoderData Encoder::fetchEncoderData() {
		
		EncoderData returnValue;
		
		leftSpeed += (rand()%3 - 1);
		rightSpeed += (rand()%3 - 1);
		
		if (leftSpeed < 0) {
			leftSpeed = 0;
		}
		
		if (rightSpeed < 0) {
			rightSpeed = 0;
		}
		
		returnValue.leftCount = leftSpeed;
		returnValue.rightCount = rightSpeed;
		
		usleep(1000);
		
		return returnValue;
		
	}
	
}

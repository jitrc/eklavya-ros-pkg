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
		
		int random = rand()%50;
		
		if (random < 2) {
			leftSpeed ++;
		} else if (random < 4) {
			rightSpeed++;
		} else if (random < 6) {
			leftSpeed --;
		} else if (random < 8) {
			rightSpeed--;
		} else if (random < 18) {
			leftSpeed ++;
			rightSpeed++;
		} else if (random < 28) {
			leftSpeed --;
			rightSpeed--;
		}
		
		if (leftSpeed > 30) {
			leftSpeed = 30;
		}
		
		if (rightSpeed > 30) {
			rightSpeed = 30;
		}
		
		if (leftSpeed < 0) {
			leftSpeed = 0;
		}
		
		if (rightSpeed < 0) {
			rightSpeed = 0;
		}
		
		if (leftSpeed - rightSpeed > 10) {
			rightSpeed = leftSpeed - 10;
		}
		
		if (rightSpeed - leftSpeed > 10) {
			leftSpeed = rightSpeed -10;
		}
		
		returnValue.leftCount = leftSpeed;
		returnValue.rightCount = rightSpeed;
		
		usleep(50000);
		
		return returnValue;
		
	}
	
}

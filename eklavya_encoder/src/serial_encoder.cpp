#include <eklavya_encoder/encoder.h>

#include "eklavya_encoder/serial_lnx.h"

#define ENCODER_COMM_PORT "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0"
#define ENCODER_BAUD_RATE 19200

Tserial *serialConnection;

namespace encoder_space {
	
	Encoder::Encoder(int argc, char **argv) {
		
		serialConnection = new Tserial();
        serialConnection->connect(ENCODER_COMM_PORT, ENCODER_BAUD_RATE, spNONE);
        usleep(100);
        
	}
	
	EncoderData Encoder::fetchEncoderData() {
		
		EncoderData returnValue;
		char input = ' ';
		
		returnValue.leftCount = 0;
		returnValue.rightCount = 0;
		
		do {
			input = serialConnection->getChar();
			if (input < 58 && input > 47) {
				returnValue.leftCount *= 10;
				returnValue.leftCount += input - 48;
			}
		} while (input != ' ');
		
		do {
			input = serialConnection->getChar();
			if (input < 58 && input > 47) {
				returnValue.rightCount *= 10;
				returnValue.rightCount += input - 48;
			}
		} while (input != ' ');
		
		return returnValue;
		
	}
	
}

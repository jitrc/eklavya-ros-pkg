#ifndef _ENCODER_H
#define _ENCODER_H

#include "../msg_gen/cpp/include/eklavya_encoder/Encoder_Data.h"

#include "eklavya_encoder/serial_lnx.h"

#define ENCODER_COMM_PORT "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0"
#define ENCODER_BAUD_RATE 19200

namespace encoder_space {
	
	class EncoderData {
		
		public:
		int leftCount;
		int rightCount;
		
	};
	
	class Encoder {
		
		
		private:
		Tserial *serialConnection;
		
		public:
		Encoder();
		EncoderData fetchEncoderData();
	};
}

#endif

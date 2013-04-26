#ifndef _ENCODER_H
#define _ENCODER_H

#include "../msg_gen/cpp/include/eklavya_encoder/Encoder_Data.h"

namespace encoder_space {
	
	class EncoderData {
		
		public:
		int leftCount;
		int rightCount;
		
	};
	
	class Encoder {		
		public:
		Encoder(int argc, char **argv);
		EncoderData fetchEncoderData();
	};
}

#endif

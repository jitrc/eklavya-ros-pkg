#include "encoder.h"

void *encoder_thread(void *arg) {
    
    encoder_space::Encoder encoder(ENCODER_COM_PORT, ENCODER_BAUD_RATE);
    encoder_space::EncoderData encoderData;
    
    while (ros::ok()) {
        /* Fetch data from Shaft Encoder and load it in local vars */
        encoderData = encoder.fetchEncoderData();

        pthread_mutex_lock(&pose_mutex);

        /* Update the pose_data using the data in local vars */

        pthread_mutex_unlock(&pose_mutex);

        usleep(10);
    }

    return NULL;
}


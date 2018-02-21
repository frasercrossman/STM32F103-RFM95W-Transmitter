//
// Created by fraser on 20/02/18.
//

#ifndef STM32F103_RFM95W_TRANSMITTER_PACKET_CONSTRUCTOR_H
#define STM32F103_RFM95W_TRANSMITTER_PACKET_CONSTRUCTOR_H

#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>

void reverse_array(unsigned char* a, int array_length);

void construct_payload(unsigned char* arr, sensors_vec_t* orientation, int time, TinyGPSPlus gps);

#endif //STM32F103_RFM95W_TRANSMITTER_PACKET_CONSTRUCTOR_H

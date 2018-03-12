//
// Created by fraser on 20/02/18.
//

#ifndef STM32F103_RFM95W_TRANSMITTER_PACKET_CONSTRUCTOR_H
#define STM32F103_RFM95W_TRANSMITTER_PACKET_CONSTRUCTOR_H

#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>

void reverse_array(unsigned char* a, int array_length);

int construct_payload_1(unsigned char* arr, int time, int power, TinyGPSPlus gps);
int construct_payload_2(unsigned char* arr, int time, int power, sensors_vec_t* orientation);

#endif //STM32F103_RFM95W_TRANSMITTER_PACKET_CONSTRUCTOR_H

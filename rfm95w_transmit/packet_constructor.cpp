//
// Created by fraser on 19/02/18.
//

#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>

#include "packet_constructor.h"
#include "packet_structure.h"

void reverse_array(unsigned char* a, int array_length) {
    int i = array_length - 1;
    int j = 0;
    int temp = 0;

    while(i > j)
    {
        temp = a[i];
        a[i] = a[j];
        a[j] = temp;
        i--;
        j++;
    }
}

void construct_block(unsigned char* arr, unsigned char block_id, unsigned char data_length, void * data) {
    arr[0] = block_id;
    arr[1] = data_length;
    memcpy(&arr[2], data, data_length);

    // Correct data endianness
    reverse_array(&arr[2], data_length);
}

void preparePayload(unsigned char* custom_payload, sensors_vec_t* orientation, int time, TinyGPSPlus gps) {
    custom_payload[0] = 0xDA;
    custom_payload[1] = 0x01;

    // IMU SEGMENT
    custom_payload[2] = 0xAA;   // Segment Flag
    custom_payload[3] = 0x00;
    custom_payload[4] = 0x02;   // Section ID IMU
    construct_block(&custom_payload[5], FLOAT_BLOCK_ID, FLOAT_BLOCK_LENGTH, &orientation->roll);
    construct_block(&custom_payload[11], FLOAT_BLOCK_ID, FLOAT_BLOCK_LENGTH, &orientation->pitch);
    construct_block(&custom_payload[17], FLOAT_BLOCK_ID, FLOAT_BLOCK_LENGTH, &orientation->heading);

    // TIME SEGMENT
    custom_payload[23] = 0xAA;   // Segment Flag
    custom_payload[24] = 0x00;
    custom_payload[25] = 0x04;   // Section ID Time
    construct_block(&custom_payload[26], INT_BLOCK_ID, INT_BLOCK_LENGTH, &time);

    // GPS SEGMENT
    custom_payload[32] = 0xAA;   // Segment Flag
    custom_payload[33] = 0x00;
    custom_payload[34] = 0x03;   // Section ID GPS

    double lat = gps.location.lat();
    double lng = gps.location.lng();

    construct_block(&custom_payload[35], DOUBLE_BLOCK_ID, DOUBLE_BLOCK_LENGTH, &lat);
    construct_block(&custom_payload[45], DOUBLE_BLOCK_ID, DOUBLE_BLOCK_LENGTH, &lng);
}
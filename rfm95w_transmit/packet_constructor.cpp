//
// Created by fraser on 19/02/18.
//

#include "packet_constructor.hpp"
#include "packet_structure.hpp"

void reverse_array(unsigned char* arr, int array_length) {
    int i = array_length - 1;
    int j = 0;
    int temp = 0;

    while(i > j)
    {
        temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
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

void construct_segment_header(unsigned char* arr, unsigned char section_id) {
    arr[0] = SEGMENT_START;
    arr[1] = 0x00;
    arr[2] = section_id;
}

void construct_battery_segment(unsigned char* arr, int battery) {
    construct_segment_header(&arr[0], BATTERY_SEG_ID);
    construct_block(&arr[0 * (SHORT_BLOCK_LENGTH + 2) + 3], SHORT_BLOCK_ID, SHORT_BLOCK_LENGTH, &battery);
}

void construct_imu_segment(unsigned char* arr, sensors_vec_t* orientation) {
    float roll = orientation->roll;
    float pitch = orientation->pitch;
    float heading = orientation->heading;

    construct_segment_header(&arr[0], IMU_SEG_ID);
    construct_block(&arr[0 * (FLOAT_BLOCK_LENGTH + 2) + 3], FLOAT_BLOCK_ID, FLOAT_BLOCK_LENGTH, &roll);
    construct_block(&arr[1 * (FLOAT_BLOCK_LENGTH + 2) + 3], FLOAT_BLOCK_ID, FLOAT_BLOCK_LENGTH, &pitch);
    construct_block(&arr[2 * (FLOAT_BLOCK_LENGTH + 2) + 3], FLOAT_BLOCK_ID, FLOAT_BLOCK_LENGTH, &heading);
}

void construct_gps_segment(unsigned char* arr, TinyGPSPlus gps) {
    double lat = gps.location.lat();
    double lng = gps.location.lng();
    double alt = 100.0;

    construct_segment_header(&arr[0], GPS_SEG_ID);
    construct_block(&arr[0 * (DOUBLE_BLOCK_LENGTH + 2) + 3], DOUBLE_BLOCK_ID, DOUBLE_BLOCK_LENGTH, &lat);
    construct_block(&arr[1 * (DOUBLE_BLOCK_LENGTH + 2) + 3], DOUBLE_BLOCK_ID, DOUBLE_BLOCK_LENGTH, &lng);
    construct_block(&arr[2 * (DOUBLE_BLOCK_LENGTH + 2) + 3], DOUBLE_BLOCK_ID, DOUBLE_BLOCK_LENGTH, &alt);
}

void construct_time_segment(unsigned char* arr, int time) {
    construct_segment_header(&arr[0], TIME_SEG_ID);
    construct_block(&arr[0 * (INT_BLOCK_LENGTH + 2) + 3], INT_BLOCK_ID, INT_BLOCK_LENGTH, &time);
}

int construct_payload_1(unsigned char* arr, int time, int power, TinyGPSPlus gps) {
    int packet_length = 2;
    arr[0] = PACKET_START_1;
    arr[1] = PACKET_START_2;

    if (time != 0) {
        construct_time_segment(&arr[packet_length], time);         // Size = 3 (segment header) + 6 * 1  (blocks) =  9 bytes
        packet_length += 9;
    }
    if (power != 0) {
        construct_battery_segment(&arr[packet_length], power);     // Size = 3 (segment header) + 4 * 1  (blocks) =  7 bytes
        packet_length += 7;
    }
    if (gps.location.lat() != 0) {
        construct_gps_segment(&arr[packet_length], gps);           // Size = 3 (segment header) + 10 * 3 (blocks) = 33 bytes
        packet_length += 33;
    }

    return packet_length; // 2 + 9 + 7 + 33 = 51 bytes
}

int construct_payload_2(unsigned char* arr, int time, int power, sensors_vec_t* orientation) {
    int packet_length = 2;
    arr[0] = PACKET_START_1;
    arr[1] = PACKET_START_2;

    if (time != 0) {
        construct_time_segment(&arr[packet_length], time);         // Size = 3 (segment header) + 6 * 1  (blocks) =  9 bytes
        packet_length += 9;
    }
    if (power != 0) {
        construct_battery_segment(&arr[packet_length], power);     // Size = 3 (segment header) + 4 * 1  (blocks) =  7 bytes
        packet_length += 7;
    }
    if (orientation->roll != 0) {
        construct_imu_segment(&arr[packet_length], orientation);    // Size = 3 (segment header) + 6 * 3  (blocks) = 21 bytes
        packet_length += 21;
    }

    return packet_length; // 2 + 9 + 7 + 21 = 39 bytes
}
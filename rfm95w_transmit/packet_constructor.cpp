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

void preparePayload(unsigned char* custom_payload, sensors_vec_t* orientation, int time, TinyGPSPlus gps) {
    custom_payload[0] = 0xDA;
    custom_payload[1] = 0x01;
    custom_payload[2] = 0xAA;   // Segment Flag
    custom_payload[3] = 0x00;
    custom_payload[4] = 0x02;   // Section ID IMU

    // Block 1, Roll
    custom_payload[5] = 0x05;   // Block ID - Float
    custom_payload[6] = 0x04;   // Data Length
    memcpy(&custom_payload[7], &orientation->roll, 4);
    reverse_array(&custom_payload[7], 4);

    // Block 2, Pitch
    custom_payload[11] = 0x05;   // Block ID - Float
    custom_payload[12] = 0x04;  // Data Length
    memcpy(&custom_payload[13], &orientation->pitch, 4);
    reverse_array(&custom_payload[13], 4);

    // Block 3, Heading
    custom_payload[17] = 0x05;  // Block ID - Float
    custom_payload[18] = 0x04;  // Data Length
    memcpy(&custom_payload[19], &orientation->heading, 4);
    reverse_array(&custom_payload[19], 4);

    custom_payload[23] = 0xAA;   // Segment Flag
    custom_payload[24] = 0x00;
    custom_payload[25] = 0x04;   // Section ID Time

    // Block 1, Time
    custom_payload[26] = 0x03;  // Block ID - Int
    custom_payload[27] = 0x04;  // Data Length
    memcpy(&custom_payload[28], &time, 4);
    reverse_array(&custom_payload[28], 4);

    custom_payload[32] = 0xAA;   // Segment Flag
    custom_payload[33] = 0x00;
    custom_payload[34] = 0x03;   // Section ID GPS

    double lat = 51.908207; //gps->location.lat();
    double lng = -1.403191; //gps->location.lng();

    // Block 1, Latitude
    custom_payload[35] = 0x06;  // Block ID - Int
    custom_payload[36] = 0x08;  // Data Length
    memcpy(&custom_payload[37], &lat, 8);
    reverse_array(&custom_payload[37], 8);

    // Block 2, Longitude
    custom_payload[45] = 0x06;  // Block ID - Int
    custom_payload[46] = 0x08;  // Data Length
    memcpy(&custom_payload[47], &lng, 8);
    reverse_array(&custom_payload[47], 8);

    // Block 3, Altitude
    //custom_payload[57] = 0x06;  // Block ID - Int
    //custom_payload[58] = 0x08;  // Data Length
    //memcpy(&custom_payload[59], &gps.location.lat(), 8); // note- change from lat to alt
    //reverse_array(&custom_payload[59], 8);
}
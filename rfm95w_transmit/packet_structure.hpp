//
// Created by fraser on 20/02/18.
//

#ifndef STM32F103_RFM95W_TRANSMITTER_PACKET_STRUCTURE_H
#define STM32F103_RFM95W_TRANSMITTER_PACKET_STRUCTURE_H

/*!
 * ============================================================================
 * General
 * ============================================================================
 */
#define PACKET_START_1          0xDA
#define PACKET_START_2          0x01

#define SEGMENT_START           0xAA

/*!
 * ============================================================================
 * Segment IDs
 * ============================================================================
 */
#define BATTERY_SEG_ID          0x01
#define IMU_SEG_ID              0x02
#define GPS_SEG_ID              0x03
#define TIME_SEG_ID             0x04

/*!
 * ============================================================================
 * Block IDs
 * ============================================================================
 */
#define BLOB_BLOCK_ID           0x00

#define BYTE_BLOCK_ID           0x01
#define SHORT_BLOCK_ID          0x02
#define INT_BLOCK_ID            0x03
#define LONG_BLOCK_ID           0x04

#define FLOAT_BLOCK_ID          0x05
#define DOUBLE_BLOCK_ID         0x06

#define STRING_BLOCK_ID         0x10

/*!
 * ============================================================================
 * Block Lengths
 * ============================================================================
 */
#define BLOB_BLOCK_LENGTH       0x00 // variable length?

#define BYTE_BLOCK_LENGTH       0x01
#define SHORT_BLOCK_LENGTH      0x02
#define INT_BLOCK_LENGTH        0x04
#define LONG_BLOCK_LENGTH       0x08

#define FLOAT_BLOCK_LENGTH      0x04
#define DOUBLE_BLOCK_LENGTH     0x08

#define STRING_BLOCK_LENGTH     0x00 // variable length?

#endif //STM32F103_RFM95W_TRANSMITTER_PACKET_STRUCTURE_H

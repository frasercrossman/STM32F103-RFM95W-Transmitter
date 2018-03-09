#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Simple_AHRS.h>
#include <TinyGPS++.h>
#include <RTClock.h>

#include "packet_constructor.hpp"

// The TinyGPS++ object
TinyGPSPlus gps;

// Initialise real-time clock
RTClock rt (RTCSEL_LSE);

// Create sensor instances.
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified mag(30302);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS ahrs(&accel, &mag);

// Orientation vector
sensors_vec_t orientation;

/*
 * Based on work by Thomas Telkamp
 * Available at: https://github.com/tftelkamp/sx127x_tx_rx
 */

#define SSPIN   PA4  // 10 for mjs board
#define DIO0    PB1  //  2 for mjs board
#define RST     PB0

#define TXDELAY 1000  // in milliseconds

/* --------------------------- */

// Set center frequency
uint32_t  freq = 868100000; // in Mhz! (868.1)

// Set tx payload
char payload[] = "1234567890ABCDEFGHIJKLMNOPQRSTUVWXYZ";

int payloadlength = sizeof(payload);
char message[256];

/* --------------------------- */

// Register mappings also apply to RFM95 devices
#include "sx1276Regs-LoRa.h"

#include <SPI.h>

// MODES
#define MODE_RX_CONTINOUS (RFLR_OPMODE_LONGRANGEMODE_ON | RFLR_OPMODE_RECEIVER)    // 0x85
#define MODE_TX           (RFLR_OPMODE_LONGRANGEMODE_ON | RFLR_OPMODE_TRANSMITTER) // 0x83
#define MODE_SLEEP        (RFLR_OPMODE_LONGRANGEMODE_ON | RFLR_OPMODE_SLEEP)       // 0x80
#define MODE_STANDBY      (RFLR_OPMODE_LONGRANGEMODE_ON | RFLR_OPMODE_STANDBY)     // 0x81

#define FRF_MSB           0xD9 // 868.1 Mhz (see datasheet)
#define FRF_MID           0x06
#define FRF_LSB           0x66

#define LNA_MAX_GAIN      (RFLR_LNA_GAIN_G1 | RFLR_LNA_BOOST_HF_ON) // 0x23
#define LNA_OFF_GAIN      RFLR_LNA_BOOST_HF_OFF



#define PA_MAX_BOOST      0x8F
#define PA_LOW_BOOST      0x81
#define PA_MED_BOOST      0x8A
#define PA_OFF_BOOST      0x00

#define MODEMCONFIG1      (RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF | RFLR_MODEMCONFIG1_CODINGRATE_4_5 | RFLR_MODEMCONFIG1_BW_125_KHZ)
#define MODEMCONFIG2      (RFLR_MODEMCONFIG2_SF_8 | RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_OFF| RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON)

void setup() {
    // initialise the pins

    // LED flash
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, LOW);
    delay(1000);
    digitalWrite(PC13, HIGH);
    delay(2000);

    pinMode(SSPIN, OUTPUT);
    pinMode(DIO0,  INPUT_PULLDOWN);
    pinMode(RST,  INPUT_PULLUP);
    digitalWrite(RST, LOW);
    delay(100);
    digitalWrite(RST, HIGH);
    delay(100);
    Serial.begin(9600);
    Serial1.begin(9600);

    SPI.begin();
    SPI.setBitOrder(MSBFIRST);            // Set the SPI_1 bit order
    SPI.setDataMode(SPI_MODE0);           // Set the  SPI_2 data mode 0
    SPI.setClockDivider(SPI_CLOCK_DIV16); // Slow speed (72 / 16 = 4.5 MHz SPI_1 speed)

    Serial.println(readRegister(0x42));
    // LoRa mode
    writeRegister(REG_LR_OPMODE,MODE_SLEEP);

    // Frequency
    //writeRegister(REG_LR_FRFMSB,FRF_MSB);
    //writeRegister(REG_LR_FRFMID,FRF_MID);
    //writeRegister(REG_LR_FRFLSB,FRF_LSB);

    // Frequency
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeRegister(REG_LR_FRFMSB, (uint8_t)(frf>>16) );
    writeRegister(REG_LR_FRFMID, (uint8_t)(frf>> 8) );
    writeRegister(REG_LR_FRFLSB, (uint8_t)(frf>> 0) );

    // Turn on implicit header mode and set payload length
    writeRegister(REG_LR_MODEMCONFIG1, MODEMCONFIG1);
    writeRegister(REG_LR_MODEMCONFIG2, MODEMCONFIG2);
    writeRegister(REG_LR_PARAMP,RFLR_PARAMP_0050_US);
    writeRegister(REG_LR_PAYLOADLENGTH, 128);
    writeRegister(REG_LR_SYNCWORD,0x34);  // LoRaWAN Public = 0x34, Private = 0x12

    // Change the DIO mapping to 01 so we can listen for TxDone on the interrupt
    writeRegister(REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_01);

    // Go to standby mode
    writeRegister(REG_LR_OPMODE,MODE_STANDBY);

    Serial.println("Device Setup Complete");

    // Initialise the sensors.
    accel.begin();
    mag.begin();
    // Initialise batter analogue read
    pinMode(PA0, INPUT_ANALOG);

    Serial.println("Sensor Initialisation Complete");
}

void loop() {
    delay(1000);
    //Serial.print(".");
    //getData();
    txloop();
}

void getData() {
    // Get GPS data
    while (Serial1.available() > 0) gps.encode(Serial1.read());

    // Get orientation data
    ahrs.getOrientation(&orientation);
}

unsigned char custom_payload[128];

void txloop() {
    int packet_length = construct_payload(&custom_payload[0], &orientation, rt.getTime(), gps, analogRead(PA0));

    writeRegister(REG_LR_PAYLOADLENGTH, packet_length);

    // Send
    writeRegister(REG_LR_OPMODE,MODE_STANDBY);
    writeRegister(REG_LR_FIFOTXBASEADDR , 0x00);
    writeRegister(REG_LR_FIFOADDRPTR, 0x00);

#define DEBUG 1
#if DEBUG
    Serial.print("{[Orientation] ");
    Serial.print("Roll: ");
    Serial.print(orientation.roll);
    Serial.print(", Pitch: ");
    Serial.print(orientation.pitch);
    Serial.print(", Heading: ");
    Serial.print(orientation.heading);

    Serial.print("} {[Time] ");
    Serial.print(rt.getTime());

    Serial.print("} {[GPS] ");
    Serial.print("Lat: ");
    Serial.print(gps.location.lat());
    Serial.print(", Lng: ");
    Serial.print(gps.location.lng());

    Serial.print("} {[Power] ");
    Serial.print(analogRead(PA0));
    Serial.println("}");
#endif

    select();

    SPI.transfer(REG_LR_FIFO | 0x80);
    for (int i = 0; i < packet_length; i++) {
        if (custom_payload[i] < 16) {
            Serial.print("0x0");
        } else {
            Serial.print("0x");
        }
        Serial.print(custom_payload[i], HEX);
        Serial.print(" ");
        SPI.transfer(custom_payload[i]);
    }
    Serial.println();

    unselect();

    writeRegister(REG_LR_LNA, LNA_OFF_GAIN);        // TURN LNA OFF FOR TRANSMIT
    writeRegister(REG_LR_PACONFIG, PA_MED_BOOST);   // TURN PA TO MAX POWER
    writeRegister(REG_LR_OPMODE, MODE_TX);

    //Serial.println("Wait for dio0 (txdone)");
    while(digitalRead(DIO0) == 0)  {   delay(10);  } //Serial.print(". ");
    //Serial.println("");

    // clear the flags 0x08 is the TxDone flag
    writeRegister(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE_MASK);

    delay(TXDELAY);
}

byte readRegister(byte addr)
{
    select();
    SPI.transfer(addr & 0x7F);
    byte regval = SPI.transfer(0);
    unselect();
    return regval;
}


void writeRegister(byte addr, byte value)
{
    select();
    SPI.transfer(addr | 0x80); // OR address with 10000000 to indicate write enable
    SPI.transfer(value);
    unselect();
}

void select()
{
    digitalWrite(SSPIN, LOW);
}


void unselect()
{
    digitalWrite(SSPIN, HIGH);
}


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Simple_AHRS.h>
#include <TinyGPS++.h>
#include <RTClock.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <EEPROM.h>

#define H_BYTES 0x0000
#define L_BYTES 0x0002

int read_counter() {
    int cnt = 0;
    uint16 readVal = 0;

    EEPROM.read(H_BYTES, &readVal);
    cnt |= readVal;
    cnt <<= 16;
    EEPROM.read(L_BYTES, &readVal);
    cnt |= readVal;

    return cnt;
}

int set_counter(int cnt) {
    EEPROM.write(H_BYTES, (cnt >> 16) & 0xFFFF);
    EEPROM.write(L_BYTES, (cnt >> 0) & 0xFFFF);
}

void increment_counter() {
    set_counter(read_counter() + 1);
}

#include "packet_constructor.hpp"

// The TinyGPS++ object
TinyGPSPlus gps;

// Initialise real-time clock
RTClock rt(RTCSEL_LSE);

// Create sensor instances.
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified mag(30302);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS ahrs(&accel, &mag);

// Orientation vector
sensors_vec_t orientation;

#define SSPIN   PA4
#define DIO0    PB1
#define RST     PB0

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t
NWKSKEY[16] = {
0xF7, 0x4B, 0x10, 0x96, 0xF0, 0xD3, 0xDf, 0x8B, 0x72, 0xC5, 0xA3, 0x99, 0x19, 0xA4, 0xAF, 0x73 };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM
APPSKEY[16] = {
0x00, 0xC7, 0xCA, 0x42, 0xC8, 0x46, 0x7B, 0x39, 0xE0, 0xF2, 0x52, 0xFE, 0x72, 0x47, 0x07, 0x4E };

// LoRaWAN end-device address
static const u4_t DEVADDR = 0x06BF10C8;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t * buf) {}

void os_getDevEui(u1_t * buf) {}

void os_getDevKey(u1_t * buf) {}

static uint8_t custom_payload[51];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
        .nss = SSPIN,
        .rxtx = LMIC_UNUSED_PIN,
        .rst = RST,
        .dio = {DIO0, PB10, PB11},
};

void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
                Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
                Serial.println(F("Received "));
                Serial.println(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
            }
            // No need to schedule a second task as another will be send on reboot
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t *j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        int packet_length = 0;
        // Construct packet
        Serial.println("Constructing the packet");

        // 5 Cycle Power average
        int avg_power = 0;
        for (int i = 0; i < 5; i++) {
            avg_power += analogRead(PA0);
        }
        avg_power = avg_power / 5;

        avg_power -= 3000;
        avg_power = constrain(avg_power, 0, 1023);

        // Frame counter even, send GPS, TIME, POWER
        // otherwise send IMU, TIME, POWER
        if (LMIC.seqnoUp % 2 == 0) {
            Serial.println("Sending GPS, TIME, and POWER");
            // Get GPS data
            while (Serial1.available() > 0) gps.encode(Serial1.read());

            packet_length = construct_payload_1(&custom_payload[0], rt.getTime(), avg_power, gps);
        } else {
            Serial.println("Sending IMU, TIME, and POWER");
            // Get orientation data
            ahrs.getOrientation(&orientation);

            packet_length = construct_payload_2(&custom_payload[0], rt.getTime(), avg_power, &orientation);
        }

#define DEBUG 1
#if DEBUG
        // Print all sensor data
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
        Serial.print(avg_power);
        Serial.println("}");

        // Print data packet (in hex)
        for (int i = 0; i < packet_length; i++) {
            if (custom_payload[i] < 16) {
                Serial.print("0x0");
            } else {
                Serial.print("0x");
            }
            Serial.print(custom_payload[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
#endif

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, custom_payload, packet_length, 0); // last parameters for ACK (0 or 1)

        // Quick double blink on transmit
        digitalWrite(PC13, LOW);
        delay(500);
        digitalWrite(PC13, HIGH);
        delay(500);
        digitalWrite(PC13, LOW);
        delay(500);
        digitalWrite(PC13, HIGH);

        increment_counter();

        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        //while (ss.available())
        while (Serial1.available() > 0)
            //gps.encode(ss.read());
            gps.encode(Serial1.read());
    } while (millis() - start < ms);
}

void setup() {
    // Serial textual output
    Serial.begin(9600);

    // Serial for GPS
    Serial1.begin(9600);

    // Fast start up blink
    smartDelay(1000);
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, LOW);
    delay(100);
    digitalWrite(PC13, HIGH);
    delay(100);

    // Startup delay
    smartDelay(4000);
    Serial.print(F("Starting with initial frame count: "));
    Serial.println(read_counter());

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
#endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // SOTON uses SF12 for its RX2 window.
    LMIC.dn2Dr = DR_SF12;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7, 14);

    Serial.println("Device Setup Complete");

    Serial.println("Starting Sensor Initialisation");
    // Initialise the sensors.
    accel.begin();
    mag.begin();
    // Initialise batter analogue read
    pinMode(PA0, INPUT_ANALOG);
    Serial.println("Sensor Initialisation Complete");

    Serial.print("Setting LMIC Initial Frame Count to: ");
    Serial.println(read_counter());

    LMIC.seqnoUp = read_counter();

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

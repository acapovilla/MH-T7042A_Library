/* Basic example for the Winsen MH-T7042A methane gas sensor */

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "MHT7042A.h"

#define RX_PIN 3
#define TX_PIN 2

SoftwareSerial softserial(RX_PIN, TX_PIN);

MHT7042A methane(&softserial);

void setup() {
    // Wait for serial monitor to open
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    Serial.println(F("MH-T7042A CH4 Gas Sensor Example"));

    softserial.begin(MHT7042A_DEFAULT_BAUDRATE);
    delay(200);

    Serial.println(F("<Arduino Ready>"));
}

void loop() {    
    sensors_event_t ch4_event;

    if (methane.getEvent(&ch4_event)) {
        Serial.print(F("CH4 concentration = "));
        Serial.print(ch4_event.unitless_percent);
        Serial.println(" %Vol");
    }

    delay(1000);
}

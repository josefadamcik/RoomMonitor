#include "keys.h" //this file is not versioned and should contain only ssid and password 
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Ticker.h>
#include <BH1750.h>
#include "debug.h"
#include "MeasurementProvider.h"
#include "DataReporter.h"



const byte tempSensAddr = 0x45; 
const byte ligthSensAddr = 0x23;
//1min
//  const unsigned long measurmentDelayMs = 10 * 1000; //10s
//  const int publishEveryNMeasurements = 6; //how often will be measured value reported in relatino to measurementDelay
//5min
const unsigned long measurmentDelayMs = 1 * 60 * 1000; //1m
const int publishEveryNMeasurements = 5; //how often will be measured value reported in relatino to measurementDelay
const char aioServer[] = "io.adafruit.com";
//const char aioServer[] = "192.168.178.29";
const int aioServerport = 8883; //ssl 8883, no ssl 1883;
const char ssid[] = MYSSID; //put #define MYSSID "xyz" in keys.h
const char password[] = MYPASS; //put #define MYPASS "blf" in keys.h
const char aioUsername[] = AIO_USERNAME; //put #define AIO_USERNAME "xyz" in keys.h
const char aioKey[] = AIO_KEY; //put #define AIO_KEY "xyz" in keys.h
const char tempfeed[] = AIO_USERNAME "/feeds/room-monitor.temperature";
const char humfeed[] = AIO_USERNAME "/feeds/room-monitor.humidity";
const char vccfeed[] = AIO_USERNAME "/feeds/room-monitor.vcc";
const char vccrawfeed[] = AIO_USERNAME "/feeds/room-monitor.vcc-raw";
const char vccwarning[] = AIO_USERNAME "/feeds/room-monitor.vcc-warning";
const char photovfeed[] = AIO_USERNAME "/feeds/room-monitor.light";
const char pressurefeed[] = AIO_USERNAME "/feeds/room-monitor.pressure";
const char msgWifiConnecting[] PROGMEM = "WIFI connecting to: ";


DataReporter reporter(
    WifiSetup( ssid, password ),
    ServerSetup( aioServer, aioServerport, aioUsername, aioKey ),
    FeedsSetup(
        tempfeed,
        humfeed,
        vccfeed,
        vccrawfeed,
        vccwarning,
        photovfeed,
        pressurefeed
    ),
    publishEveryNMeasurements
);

MeasurementProvider measurement(tempSensAddr, ligthSensAddr);

void setup() {
    Serial.begin(115200);
    Serial.println();
    //deep sleep state recovery, not yet used
    // uint32_t control;
    // ESP.rtcUserMemoryRead(0, &control, sizeof(uint32_t));
    // Serial.print("setup found, control: ");
    // Serial.print(control);
    // if (control == 0) {
    //     coldStart = false;
    //     ESP.rtcUserMemoryRead(4, &measurements.reportIn, sizeof(uint32_t));
    //     Serial.print(", reportIn: ");
    //     Serial.print(measurements.reportIn);
    // }
    // Serial.println();
   
    Wire.begin(/*sda*/D2, /*scl*/D1);
    if (!measurement.begin()) {
        Serial.println("Unable to initialize measurement");
        while(1);
    }
    reporter.begin();
    Serial.println(F("Starting..."));
}

void loop() {
    delay(250); //Let things settle a bit (mainly the power source voltage)
    bool measureRes = measurement.doMeasurements();
    const MeasurementsData measurementData = measurement.getCurrentMeasurements();
    measurementData.printToSerial();
    Serial.println();

    if (!measureRes) {
        Serial.println(F("Unable to measure temperature"));
    } else {
        reporter.doReport(measurementData);
    }
    Serial.println(F("Loop end."));
    delay(measurmentDelayMs); 
}





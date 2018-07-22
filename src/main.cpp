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
#include "BatteryMonitor.h"
#include "MeasurementProvider.h"
#include "DataReporter.h"
extern "C" { //esp8266 sdk, if we need to call deep sleep apis directly
    #include "user_interface.h" 
}


const byte tempSensAddr = 0x45; 
const byte ligthSensAddr = 0x23;
//1min
const unsigned long sleepForUs = 60 * 1000000; //1m
//5min
// const unsigned long sleepForUs = 5 * 60 * 1000000; //5m

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

// BatteryMonitor batteryMonitor(280,360);
BatteryMonitor batteryMonitor(340,360);

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
    &batteryMonitor
);

MeasurementProvider measurement(tempSensAddr, ligthSensAddr);

/**
 * Everything happens in setup, we have nothing in loop because we are sleeping all the time.
 */ 
void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println(F("Starting..."));
   
    Wire.begin(/*sda*/D2, /*scl*/D1);
    if (!measurement.begin()) {
        Serial.println("Unable to initialize measurement");
        while(1);
    }

    //start wifi and mqtt
    reporter.begin();
    reporter.ensureWifiConnection();
    delay(100); //Let things settle 
    
    //do measurements and report
    bool measureRes = measurement.doMeasurements();
    const MeasurementsData measurementData = measurement.getCurrentMeasurements();

    if (!measureRes) {
        Serial.println(F("Unable to measure temperature"));
    } else {
        reporter.doReport(measurementData);
        
        Serial.print(F("BatteryMonitor state: "));
        Serial.println(batteryMonitor.getState().triggered);
    }
    Serial.println(F("Loop end"));
    
    //finish thins up
    reporter.closeConnections();
    Serial.println(F("Go to sleep..."));
    Serial.flush();
    //sleep deeply
    ESP.deepSleep(sleepForUs, WAKE_RF_DEFAULT);
    delay(500);
}

void loop() {
    // NOP
}






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
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>


const byte tempSensAddr = 0x45; 
const byte ligthSensAddr = 0x23;
// //1min
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


const uint32_t deepSleepStateMagic = 0x8af2bc12;

BatteryMonitor batteryMonitor(290,360);
//BatteryMonitor batteryMonitor(340,360);

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
    delay(100);
    Serial.begin(57600);
    Serial.println();
    Serial.println(F("Starting..."));

    pinMode(D3, INPUT_PULLUP);

    //restor state from rtc memory 
    BatteryMonitorState oldState(false);
    ESP.rtcUserMemoryRead(0, reinterpret_cast<uint32_t*>(&oldState), sizeof(oldState));


    if (oldState.magic != deepSleepStateMagic) { //FIRST RUN
        Serial.println(F("First run!"));
    } else { //state restored
        Serial.print(F("Recoverd old state... battery: "));
        Serial.println(oldState.triggered);
        batteryMonitor.setState(oldState);
    }

    Wire.begin(/*sda*/D2, /*scl*/D1);
    Wire.setTimeout(500);
    if (!measurement.begin()) {
        Serial.println("Unable to initialize measurement");
        //TODO: report error, go to sleep. Don't loop.
        // while(1);
    }

    //start wifi and mqtt
    reporter.begin();
    reporter.ensureWifiConnection();
    delay(100); //Let things settle

    // Initialise OTA in case there is a software upgrade
    ArduinoOTA.setHostname("roommonitor.local");
    ArduinoOTA.onStart([]() { Serial.println("Start"); });
    ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
            Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
            Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
            Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
            Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
            Serial.println("End Failed");
    });

    ArduinoOTA.begin();

    // wait 10 secs and check if button pressed, which will trigger a wait for
    // the OTA reflash
    bool waiforOTA = false;
    Serial.println("Waiting for any OTA updates");
    int keeptrying = 5;
    while (keeptrying-- > 0 || waiforOTA == true) {
        if (digitalRead(D3) == LOW) {
            waiforOTA = true;
            Serial.print("OTA");
        }
        Serial.print(".");
        ArduinoOTA.handle();
        delay(500);
    }

    // do measurements and report
    bool measureRes = measurement.doMeasurements();
    const MeasurementsData measurementData = measurement.getCurrentMeasurements();

    if (!measureRes) {
        Serial.println(F("Unable to measure temperature"));
    } else {
        reporter.doReport(measurementData);
        
    }
    Serial.println(F("Loop end"));

    //finish things up
    reporter.closeConnections();
    Serial.print(F("Go to sleep..."));
    Serial.println(sleepForUs);
    Serial.flush();
    
    BatteryMonitorState newBatteryState = batteryMonitor.getState();
    Serial.print(F("BatteryMonitor state: "));
    Serial.println(newBatteryState.triggered);

    //writ state to rtc memory
    newBatteryState.magic = deepSleepStateMagic;
    ESP.rtcUserMemoryWrite(0, reinterpret_cast<uint32_t*>(&newBatteryState), sizeof(newBatteryState));
    delay(2);

    //sleep deeply
    ESP.deepSleep(sleepForUs, WAKE_RF_DEFAULT);
    delay(500);
}

void loop() {
    // NOP
}






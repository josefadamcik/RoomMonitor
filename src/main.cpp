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
// const unsigned long sleepForUs = 60 * 1000000; //1m
//5min
const unsigned long sleepForUs = 5 * 60 * 1000000; //5m
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
bool measurementReady = false;

IPAddress ip(192, 168, 178, 27);
IPAddress gateway(192, 168, 178, 1);
IPAddress subnet(255, 255, 255, 0);


const uint32_t deepSleepStateMagic = 0x8af2ba12;

BatteryMonitor batteryMonitor(290,360);

RoomMonitorState oldState;

DataReporter reporter(
    WifiSetup( ssid, password, ip, gateway, subnet ),
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

void otaInitialize() {
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
}

void storeState() {
    RoomMonitorState newState = reporter.getState(batteryMonitor.triggered);
    Serial.print(F("RoomMonitorState state: "));
    Serial.println(newState.triggered);
    if (newState.validWifi) {
        Serial.println(F("Valid Wifi setup"));
        Serial.print(F("Channel: "));
        Serial.println(newState.channel);
        Serial.print(F("BSSID: "));
        for (byte i = 0; i < 6; i++) {
            Serial.print(newState.bssid[i]);
            Serial.print(F(":"));
        }
        Serial.println();
    } else {
        Serial.println(F("No valid wifi setup."));
    }

    // writ state to rtc memory
    newState.magic = deepSleepStateMagic;
    ESP.rtcUserMemoryWrite(0, reinterpret_cast<uint32_t*>(&newState),
                           sizeof(newState));
    delay(2);
}

/**
 * Everything happens in setup, we have nothing in loop because we are sleeping all the time.
 */ 
void setup() {
    //switch radio off to save energy
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
    delay(100);

    Serial.begin(57600);
    Serial.println();
    Serial.println(F("Starting..."));

    pinMode(D3, INPUT_PULLUP);

    //restor state from rtc memory 

    ESP.rtcUserMemoryRead(0, reinterpret_cast<uint32_t*>(&oldState), sizeof(oldState));

    if (oldState.magic != deepSleepStateMagic) { //FIRST RUN
        Serial.println(F("First run!"));
        oldState.valid = false;
    } else { //state restored
        oldState.valid = true;
        Serial.print(F("Recoverd old state... battery: "));
        Serial.println(oldState.triggered);
        batteryMonitor.setState(oldState.triggered);
    }

    // wait a few sec for OTA button press, when the button is pressed we will be waiting for OTA update.
    bool waiforOTA = false;
    Serial.println("Waiting for any OTA updates");
    int keeptrying = 5;
    while (keeptrying-- > 0 && !waiforOTA) {
        if (digitalRead(D3) == LOW) {
            waiforOTA = true;
            Serial.println();
            Serial.println("Will wait for OTA");
            break;
        }
        Serial.print(".");
        delay(500);
    }

    if (waiforOTA) {
        reporter.begin(oldState);
        otaInitialize();
        while(1) {
            Serial.print(".");
            ArduinoOTA.handle();
            delay(500);
        }
    } 

    Serial.println("No OTA update");
    //no ota update (there will be restart after OTA or manual restart if ota was not performed)
    Wire.begin(/*sda*/D2, /*scl*/D1);
    Wire.setTimeout(500);
    measurementReady = measurement.begin();
    if (!measurementReady) {
        Serial.println("Unable to initialize measurement");
    } else {
        // do measurements
        bool measureRes = measurement.doMeasurements();
        const MeasurementsData measurementData = measurement.getCurrentMeasurements();
        if (!measureRes) {
            Serial.println(F("Unable to measure"));
        } else {
            // start wifi and mqtt
            reporter.begin(oldState);
            // report
            reporter.doReport(measurementData);
            //store state
            storeState();
            delay(100);
            // finish things up
            reporter.closeConnections();
        }
    }
    Serial.print(F("Go to sleep..."));
    Serial.println(sleepForUs);
    Serial.flush();
    //sleep deeply
    ESP.deepSleep(sleepForUs, WAKE_RF_DISABLED);
}

void loop() {
    // NOP
}






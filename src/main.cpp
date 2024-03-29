#include "keys.h" //this file is not versioned and should contain only ssid and password 
#include <Arduino.h>
#include "config.h"
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Ticker.h>
#include <BH1750.h>
#include "BatteryMonitor.h"
#include "MeasurementProvider.h"
#include "DataReporter.h"
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>

#ifdef USE_SHT21 
const byte tempSensAddr = 0x40;
#endif
#ifdef USE_SHT30
const byte tempSensAddr = 0x45;
#endif  
const byte ligthSensAddr = 0x23;
//1min
// const unsigned long sleepForUs = 60 * 1000000; //1m
//5min
const unsigned long sleepForUs = 5 * 60 * 1000000; //5m
const char mqttServer[] = "192.168.178.76";
const int mqttServerport = 1883; 
const char mqttLogin[] = "mqtt";
const char mqttPass[] = MQTTPASS;
const char ssid[] = MYSSID; //put #define MYSSID "xyz" in keys.h
const char password[] = MYPASS; //put #define MYPASS "blf" in keys.h
const char tempfeed[] = "home/" ROOM_NAME "/temperature";
const char humfeed[] = "home/" ROOM_NAME "/humidity";
const char vccfeed[] = "home/" ROOM_NAME "/vcc";
const char vccrawfeed[] = "home/" ROOM_NAME "/vcc-raw";
const char vccwarning[] = "home/" ROOM_NAME "/vcc-warning";
const char photovfeed[] = "home/" ROOM_NAME "/light";
const char pressurefeed[] = "home/" ROOM_NAME "/pressure";
const char msgWifiConnecting[] PROGMEM = "WIFI connecting to: ";
const float analogVccToRealCoeficient = 0.0047705311; // Second Prototype: 0.004904651; D1 Mini: 0.00611
bool measurementReady = false;

// IPAddress ip(192, 168, 178, 51); //bedroom 
IPAddress ip(192, 168, 178, 50);  //living-room 
IPAddress gateway(192, 168, 178, 1);
IPAddress subnet(255, 255, 255, 0);

const uint32_t deepSleepStateMagic = 0x8af2ba12;

BatteryMonitor batteryMonitor(/* trigger low voltage */ 320, /* reset low voltage */ 360);

RoomMonitorState oldState;

DataReporter reporter(
    WifiSetup( ssid, password, ip, gateway, subnet ),
    ServerSetup( mqttServer, mqttServerport, mqttLogin, mqttPass),
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

MeasurementProvider measurement(tempSensAddr, ligthSensAddr, analogVccToRealCoeficient);

void otaInitialize() {
    // Initialise OTA in case there is a software upgrade
    ArduinoOTA.setHostname(ROOM_NAME ".roommonitor.local");
    ArduinoOTA.onStart([]() { Serial.println("Start"); });
    ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) { 
            Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) { 
            Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
        }
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
    Wire.begin(/*sda*/4, /*scl*/5); //GPIO4, GPIO5
    Wire.setTimeout(500);
    measurementReady = measurement.begin();
    if (!measurementReady) {
        Serial.println("Unable to initialize measurement");
    } else {
        // do measurements
        Serial.println("Do measurements");
        bool measureRes = measurement.doMeasurements();
        const MeasurementsData measurementData = measurement.getCurrentMeasurements();
        if (!measureRes) {
            Serial.println(F("Unable to measure"));
        } else {
            Serial.println("Send measurements");
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
    ESP.deepSleepInstant(sleepForUs, WAKE_RF_DISABLED);
}

void loop() {
    // NOP
}






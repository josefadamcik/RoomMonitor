#include "keys.h" //this file is not versioned and should contain only ssid and password 
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Ticker.h>
#include <BH1750.h>
#include "debug.h"
#include "BatteryMonitor.h"
#include "MeasurementProvider.h"
#include "DataReporter.h"
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>

const byte tempSensAddr = 0x40; //sht21 Ox40, sht3; Ox45 (?)
const byte ligthSensAddr = 0x23;
// //1min
// const unsigned long sleepForUs = 60 * 1000000; //1m
//5min
const unsigned long sleepForUs = 5 * 60 * 1000000; //5m
const char mqttServer[] = "192.168.178.31";
const int mqttServerport = 1883; 
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
bool measurementReady = false;

// IPAddress ip(192, 168, 178, 33); //bedroom (old prototype)
IPAddress ip(192, 168, 178, 50);  //living-room (new pcb)
IPAddress gateway(192, 168, 178, 1);
IPAddress subnet(255, 255, 255, 0);

const uint32_t deepSleepStateMagic = 0x8af2ba12;

BatteryMonitor batteryMonitor(290,360);

RoomMonitorState oldState;

DataReporter reporter(
    WifiSetup( ssid, password, ip, gateway, subnet ),
    ServerSetup( mqttServer, mqttServerport),
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

void scanadr() {
    byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan
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
    scanadr();
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
    ESP.deepSleep(sleepForUs, WAKE_RF_DISABLED);
}



void loop() {
    // NOP
}






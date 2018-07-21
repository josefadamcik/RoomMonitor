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
#include "MeasurementProvider.h"
#include "DataReporter.h"

#define DEBUG true
#define Serial if(DEBUG)Serial

const char aioSslFingreprint[] = "77 00 54 2D DA E7 D8 03 27 31 23 99 EB 27 DB CB A5 4C 57 18";
const int powerLowerThanWarningThresholds[] = {490, 480, 470}; //round(vcc * 10)
const byte powerLowerThanWarningThresholdCount = 3;
// const int powerLowerThanWarningThresholds[] = {520, 510, 500}; //round(vcc * 10)
const bool vccReportingOn = false;

const byte tempSensAddr = 0x45; 
const byte ligthSensAddr = 0x23;
//1min
const unsigned long measurmentDelayMs = 10 * 1000; //10s
const int publishEveryNMeasurements = 6; //how often will be measured value reported in relatino to measurementDelay
//5min
// const unsigned long measurmentDelayMs = 1 * 60 * 1000; //1m
// const int publishEveryNMeasurements = 5; //how often will be measured value reported in relatino to measurementDelay
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


MeasurementProvider measurement(tempSensAddr, ligthSensAddr);


WiFiClientSecure client;
//WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, aioServer, aioServerport, aioUsername, aioKey);
Adafruit_MQTT_Publish mqttTempFeed = Adafruit_MQTT_Publish(&mqtt,tempfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttHumFeed = Adafruit_MQTT_Publish(&mqtt,humfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttVccFeed = Adafruit_MQTT_Publish(&mqtt,vccfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttVccRawFeed = Adafruit_MQTT_Publish(&mqtt,vccrawfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttVccWarning = Adafruit_MQTT_Publish(&mqtt,vccwarning, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttPhotoVFeed = Adafruit_MQTT_Publish(&mqtt,photovfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttPressureFeed = Adafruit_MQTT_Publish(&mqtt,pressurefeed, MQTT_QOS_1);

void MQTTConect();
void MQTTDisconnect();
void WIFIConect(bool debugBlink);
void WIFIshowConnecting();
void WIFIShowConnected();
void verifyFingerprint();
byte measureTemp();


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
    Serial.println(F("Starting..."));
    WiFi.persistent(false);
    // WiFi.setSleepMode(WIFI_MODEM_SLEEP); //light sleep is more than default (modem sleep)
    // WiFi.setSleepMode(WIFI_LIGHT_SLEEP); //light sleep is more than default (modem sleep)
}

bool coldStart = true;

void loop() {
    MQTTConect();
    delay(100); //Let things settle a bit (mainly the power source voltage)
    bool measureRes = measurement.doMeasurements();
    const MeasurementsData measurementData = measurement.getCurrentMeasurements();
    measurementData.printToSerial();
    Serial.println();

    if (!measureRes) {
        Serial.println(F("Unable to measure temperature"));
    } else {
        if (measurementData.reportIn == 0) { //downcounter reached zero, we are publishing to mqtt
            // MQTTConect();
            Serial.print(F("Sending measurements: "));
            measurementData.printToSerial();
            Serial.print(F("... "));
            //debug - dont send vaues
            // bool succ = true;
            bool succ = mqttTempFeed.publish(measurementData.temperature);
            succ = mqttHumFeed.publish(measurementData.humidity) && succ;
            const float reportedVoltage = measurementData.voltage;
            succ = mqttVccFeed.publish(reportedVoltage) && succ;
            succ = mqttVccRawFeed.publish(measurementData.voltageRaw) && succ;
            succ = mqttPhotoVFeed.publish(measurementData.lightLevel) && succ;
            succ = mqttPressureFeed.publish(measurementData.pressure) && succ;
            //Should we send power warning? We are sending value only when it goes under some threshold 
            //for the first time.
            //detect actual threshold
            const int thresholdValue = roundf(reportedVoltage * 100.0f);
            Serial.print(F("Computed threshold: "));
            Serial.print(thresholdValue);
            Serial.print(F(" last: "));
            Serial.println(measurementData.lastPowerWarningThreshold);
            int currentThreshold = 0;
            for (unsigned char i = 0; i < powerLowerThanWarningThresholdCount; i++) {
                if (thresholdValue <= powerLowerThanWarningThresholds[i]) {
                    currentThreshold = powerLowerThanWarningThresholds[i];
                }
            }

            if (vccReportingOn 
                    && !coldStart 
                    && currentThreshold > 0 
                    && currentThreshold != measurementData.lastPowerWarningThreshold) {
                Serial.print(F("Reporting threshold: "));
                Serial.print(currentThreshold);
                mqttVccWarning.publish(currentThreshold);
                //measurementData.lastPowerWarningThreshold = currentThreshold; //TODO
            }

            //success, we will reset counter, failur wont'
            if (succ) {
                //measurementData.reportIn = publishEveryNMeasurements - 1; //TODO
                Serial.println(F(" OK!"));
            } else {
                Serial.println(F(" Failed"));
            }
//            MQTTDisconnect();
      } else {
            //count down
            //measurementData.reportIn--; //TODO:
        }
    }
    
    coldStart =  false;
    delay(measurmentDelayMs); 
}



void MQTTConect() {
    int wifiStatus = WiFi.status();
    if (wifiStatus != WL_CONNECTED){
        Serial.println();
        Serial.print(F("WiFi not connected, status: "));
        Serial.print(wifiStatus);
        Serial.println();
        WIFIshowConnecting();
        WiFi.mode(WIFI_STA);
        //WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
        if (wifiStatus == WL_DISCONNECTED || wifiStatus == WL_CONNECTION_LOST) {
            WiFi.reconnect();
        } else {
            WiFi.begin(ssid, password);
        }
        
        while (WiFi.status() != WL_CONNECTED) {
            delay(250);
            delay(250);
            Serial.print(WiFi.status());
        }
        Serial.println();
        WIFIShowConnected();
        verifyFingerprint();
    }  

    int8_t ret;

    // Stop if already connected, but double check with ping
    if (mqtt.connected()) {
        if (mqtt.ping()) {
            return;
        } else {
            Serial.println(F("Connected was true, but ping returned false"));
            mqtt.disconnect();
        }
    }
    Serial.print(F("Connecting to MQTT... "));

    uint8_t retries = 3;
    while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println(F("Retr MQTT connection in 5 seconds..."));
        mqtt.disconnect();
        delay(5000);  // wait 5 seconds
        retries--;
        if (retries == 0) {
            // basically die and wait for WDT to reset me
            while (1);
        }
    }   

    if (mqtt.connected()) {
        Serial.println(F(" MQTT Connected!"));
    } else {
        Serial.print(F(" MQTT still NOT onnected! "));
        Serial.println(ret);
    }
}

void MQTTDisconnect() {
    if (mqtt.connected()) {
        mqtt.disconnect();
    }
    int wifiStatus = WiFi.status();
    if(wifiStatus == WL_CONNECTED) {
        WiFi.disconnect();
    }
    if (WiFi.getMode() != WIFI_OFF ) {
        Serial.println("Wifi off...");
        WiFi.mode(WIFI_OFF); //really turn off -> without this it would actually consume more power than when connected
        WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
    }
    
}


void WIFIshowConnecting() {
    Serial.print(FPSTR(msgWifiConnecting));
    Serial.println(ssid);
}

void WIFIShowConnected() {
    Serial.print(F("Your ESP is connected! Your IP address is: "));  
    Serial.println(WiFi.localIP());      
}

void WIFIConect(bool debugBlink) {
    WIFIshowConnecting();    
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        delay(250);
        Serial.print(".");
    }   
    Serial.println(F("Setup done"));
    WIFIShowConnected();
    verifyFingerprint();
}


void verifyFingerprint() {
  const char* host = aioServer;
  Serial.println(host);
  if (! client.connect(host, aioServerport)) {
    Serial.println(F("Connection failed."));
    while(1);
  }
  if (client.verify(aioSslFingreprint, host)) {
    Serial.println(F("Connection secure."));
  } else {
    Serial.println(F("Connection insecure!"));
    while(1);
  }

  client.stop(); //otherwise the MQTT.connected() will return true, because the implementation
  // just asks the client if there is a connectioni. It actully doesn't check if there was a mqtt connection established.

}
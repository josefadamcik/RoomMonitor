#include <Arduino.h>
#include "ESP8266WiFi.h"
#include "keys.h" //this file is not versioned and should contain only ssid and password 

const char aio_server[] PROGMEM = "io.adafruit.com";
const char aio_serverport[] PROGMEM = 1883;

const char ssid[]  PROGMEM = MYSSID; //put #define MYSSID "xyz" in keys.h
const char password[]  PROGMEM = MYPASS; //put #define MYPASS "blf" in keys.h
const char aio_username[]  PROGMEM = AIO_USERNAME;
const char aio_key[] PROGMEM = AIO_KEY;


void connectToWifi(bool debugBlink) {
    Serial.print(F("Your are connecting to;"));
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        if (debugBlink) {
            digitalWrite(16, HIGH);
        }
        delay(250);
        if (debugBlink) {
            digitalWrite(16, LOW);
        }
        delay(250);
        Serial.print(".");
    }   
    Serial.println(F("Setup done"));
}

void setup() {
    Serial.begin(115200);
    pinMode(16, OUTPUT);
    connectToWifi(true);
}


void loop() {
      int wifiStatus = WiFi.status();

      if(wifiStatus == WL_CONNECTED){
         Serial.println(F(""));
         Serial.println(F("Your ESP is connected!"));  
         Serial.println(F("Your IP address is: "));
         Serial.println(WiFi.localIP());  
      }
      else{
        Serial.println(F(""));
        Serial.println(F("WiFi not connected"));
      }
      delay(1000); // check for connection every once a second
}
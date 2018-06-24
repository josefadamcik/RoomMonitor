#include <Arduino.h>
#include "ESP8266WiFi.h"
#include "keys.h" //this file is not versioned and should contain only ssid and password 
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define AIO_SSL_FINGERPRINT "77 00 54 2D DA E7 D8 03 27 31 23 99 EB 27 DB CB A5 4C 57 18"

const char aio_server[] = "io.adafruit.com";
//const char aio_server[] = "192.168.178.29";
const int aio_serverport = 8883; //ssl 8883, no ssl 1883;

const char ssid[] = MYSSID; //put #define MYSSID "xyz" in keys.h
const char password[] = MYPASS; //put #define MYPASS "blf" in keys.h
const char aio_username[] = AIO_USERNAME;
const char aio_key[] = AIO_KEY;
const char feed[] = "josefadamcik/feeds/test";


WiFiClientSecure client;
//WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, aio_server, aio_serverport, aio_username, aio_key);
Adafruit_MQTT_Publish mqttFeed = Adafruit_MQTT_Publish(&mqtt,feed, MQTT_QOS_1);


void MQTT_connect();
void WIFI_connect(bool debugBlink);
//void verifyFingerprint();

void WIFI_connect(bool debugBlink) {
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
    Serial.print(F("Your ESP is connected! Your IP address is: "));  
    Serial.println(WiFi.localIP());      
}

void setup() {
    Serial.begin(115200);
    pinMode(16, OUTPUT);
    WiFi.mode(WIFI_STA);
    WIFI_connect(true);
    //verifyFingerprint();
}

int x = 0;
void loop() {
    MQTT_connect();
    
    // Now we can publish stuff!
    Serial.print(F("\nSending photocell val "));
    Serial.print(x);
    Serial.print("...");
    
    if (! mqttFeed.publish(x++)) {
        Serial.println(F("Failed"));
    } else {
        Serial.println(F("OK!"));
    }
    // if(! mqtt.ping()) {
    //     mqtt.disconnect();
    // }
    delay(3000); 
}


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
    int wifiStatus = WiFi.status();
    if(wifiStatus != WL_CONNECTED){
        Serial.println();
        Serial.print(F("WiFi not connected, status: "));
        Serial.print(wifiStatus);
        Serial.println();
        WiFi.reconnect();
        while (WiFi.status() != WL_CONNECTED) {
            digitalWrite(16, HIGH);
            delay(250);
            digitalWrite(16, LOW);
            delay(250);
            Serial.print(WiFi.status());
        }   
    }  

    int8_t ret;

    // Stop if already connected.
    if (mqtt.connected()) {
        return;
    }

    Serial.print(F("Connecting to MQTT... "));
    //Serial.print(feed);

    uint8_t retries = 3;
    while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println(F(" Retrying MQTT connection in 5 seconds..."));
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



// void verifyFingerprint() {

//   const char* host = aio_server;

//   Serial.print("Connecting to ");
//   Serial.println(host);

//   if (! client.connect(host, aio_serverport)) {
//     Serial.println("Connection failed. Halting execution.");
//     while(1);
//   }

//   if (client.verify(AIO_SSL_FINGERPRINT, host)) {
//     Serial.println("Connection secure.");
//   } else {
//     Serial.println("Connection insecure! Halting execution.");
//     while(1);
//   }

// }
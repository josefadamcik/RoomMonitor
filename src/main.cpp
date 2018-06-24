#include <Arduino.h>
#include "ESP8266WiFi.h"
#include "Wire.h"
#include "keys.h" //this file is not versioned and should contain only ssid and password 
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

const char aioSslFingreprint[] "77 00 54 2D DA E7 D8 03 27 31 23 99 EB 27 DB CB A5 4C 57 18";

const byte tempSensAddr = 0x45; 
const int ledPin = D0;
const char aioServer[] = "io.adafruit.com";
//const char aioServer[] = "192.168.178.29";
const int aioServerport = 8883; //ssl 8883, no ssl 1883;

const char ssid[] = MYSSID; //put #define MYSSID "xyz" in keys.h
const char password[] = MYPASS; //put #define MYPASS "blf" in keys.h
const char aioUsername[] = AIO_USERNAME; //put #define AIO_USERNAME "xyz" in keys.h
const char aioKey[] = AIO_KEY; //put #define AIO_KEY "xyz" in keys.h
const char tempfeed[] = AIO_USERNAME "/feeds/room-monitor.temperature";
const char humfeed[] = AIO_USERNAME "/feeds/room-monitor.humidity";

struct Measurements {
    float temperature = 0.0;
    float humidity = 0.0;
} measurements;


WiFiClientSecure client;
//WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, aioServer, aioServerport, aioUsername, aioKey);
Adafruit_MQTT_Publish mqttTempFeed = Adafruit_MQTT_Publish(&mqtt,tempfeed, MQTT_QOS_0);
Adafruit_MQTT_Publish mqttHumFeed = Adafruit_MQTT_Publish(&mqtt,humfeed, MQTT_QOS_0);


void MQTT_connect();
void WIFI_connect(bool debugBlink);
void verifyFingerprint();
byte measureTemp();

void WIFI_connect(bool debugBlink) {
    Serial.print(F("Your are connecting to;"));
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        if (debugBlink) {
            digitalWrite(ledPin, HIGH);
        }
        delay(250);
        if (debugBlink) {
            digitalWrite(ledPin, LOW);
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
    pinMode(ledPin, OUTPUT);
    WiFi.mode(WIFI_STA);
    Wire.begin(/*sda*/D2, /*scl*/D1);
    WIFI_connect(true);
    verifyFingerprint();
}

int x = 0;
void loop() {
    MQTT_connect();
    // Now we can publish stuff!
    byte measureRes = measureTemp(); 
    if (measureRes != 0) {
        Serial.println(F("Unable to measure temperature"));
        Serial.println(measureRes);
    } else {
        Serial.print(F("Sending measurements, temp: "));
        Serial.print(measurements.temperature);
        Serial.print(F(", hum: "));
        Serial.print(measurements.humidity);
        Serial.print(F("... "));
        bool succ = mqttTempFeed.publish(measurements.temperature);
        succ = mqttHumFeed.publish(measurements.humidity) && succ;
        if (succ) {
            Serial.println(F("OK!"));
        } else {
            Serial.println(F("Failed"));
        }
    }
    
    // if(! mqtt.ping()) {
    //     mqtt.disconnect();
    // }
    delay(5000); 
}

byte measureTemp() {
    unsigned int data[6];

    // Start I2C Transmission
    Wire.beginTransmission(tempSensAddr);
    // Send measurement command
    Wire.write(0x2C);
    Wire.write(0x06);
    // Stop I2C transmission
    if (Wire.endTransmission() != 0)  {
        return 1;
    }
    delay(500);
    // Request 6 bytes of data
    Wire.requestFrom(tempSensAddr, 6);
    // Read 6 bytes of data
    // cTemp msb, cTemp lsb, cTemp crc, humidity msb, humidity lsb, humidity crc
    for (int i=0;i<6;i++) {
        data[i]=Wire.read();
    };

    delay(50);

    if (Wire.available() != 0) {
        return 2;
    }
    // Convert the data
    measurements.temperature = ((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45;
    measurements.humidity = ((((data[3] * 256.0) + data[4]) * 100) / 65535.0);

    return 0;
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



void verifyFingerprint() {

  const char* host = aioServer;

  Serial.print("Connecting to ");
  Serial.println(host);

  if (! client.connect(host, aioServerport)) {
    Serial.println("Connection failed. Halting execution.");
    while(1);
  }

  if (client.verify(aioSslFingreprint, host)) {
    Serial.println("Connection secure.");
  } else {
    Serial.println("Connection insecure! Halting execution.");
    while(1);
  }

}
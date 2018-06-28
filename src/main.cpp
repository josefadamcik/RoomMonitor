#include <Arduino.h>
#include "ESP8266WiFi.h"
#include "Wire.h"
#include "keys.h" //this file is not versioned and should contain only ssid and password 
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <LiquidCrystal_I2C.h>
#include <Ticker.h>

const char aioSslFingreprint[] = "77 00 54 2D DA E7 D8 03 27 31 23 99 EB 27 DB CB A5 4C 57 18";

const byte tempSensAddr = 0x45; 
const byte displayAddr = 0x27;
const byte displayOnButtonPin = D3;
//const unsigned long measurmentDelayMs = 10 * 1000; //10s
const unsigned long measurmentDelayMs = 1 * 60 * 1000; //1m
const int publishEveryNMeasurements = 5; //how often will be measured value reported in relatino to measurementDelay
const unsigned long displayBacklightOnDelayS = 5;
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


const char msgWifiConnecting[] PROGMEM = "WIFI connecting to: ";

struct Measurements {
    float temperature = 0.0;
    float humidity = 0.0;
    float voltage = 0.0;
    int voltageRaw = 0;
    uint32_t reportIn = 0;
} measurements;

struct PersistentState {

} state;
bool displayingTemp = false;

LiquidCrystal_I2C lcd(displayAddr, 16, 2);

WiFiClientSecure client;
//WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, aioServer, aioServerport, aioUsername, aioKey);
Adafruit_MQTT_Publish mqttTempFeed = Adafruit_MQTT_Publish(&mqtt,tempfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttHumFeed = Adafruit_MQTT_Publish(&mqtt,humfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttVccFeed = Adafruit_MQTT_Publish(&mqtt,vccfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttVccRawFeed = Adafruit_MQTT_Publish(&mqtt,vccrawfeed, MQTT_QOS_1);


Ticker displayBacklightTicker;
volatile bool displayBacklightOn = false;
volatile unsigned long displayBacklightOnSince = 0;
boolean coldStart = true;


void MQTTConect();
void MQTTDisconnect();
void WIFIConect(bool debugBlink);
void WIFIshowConnecting();
void WIFIShowConnected();
void verifyFingerprint();
byte measureTemp();
void lcdReset();
void lcdTurnOnBacklight();
void onDisplayButtonTriggered();


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
   
    pinMode(displayOnButtonPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(displayOnButtonPin), onDisplayButtonTriggered, FALLING);
    Wire.begin(/*sda*/D2, /*scl*/D1);
    lcd.init();
    if (coldStart) {
        lcdReset();
        lcd.home();
        lcd.print(F("Starting..."));
        lcdTurnOnBacklight();
    }
    Serial.println(F("Starting..."));
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setSleepMode(WIFI_LIGHT_SLEEP); //light sleep is more than default (modem sleep)
}



void loop() {
    measurements.voltageRaw = analogRead(A0);
    //measurements.voltage = (measurements.voltageRaw / 1024.0f ) * 3.3f; //0-3.3, there's internal voltage divider 100k/220k
    measurements.voltage = (measurements.voltageRaw / 1024.0f ) * 2.81f * 2.266f; //0-6.6v, external voltage divider 220/(680 + internal divider in paralel) 
    byte measureRes = measureTemp(); 

    if (measureRes != 0) {
        Serial.println(F("Unable to measure temperature"));
        Serial.println(measureRes);
    } else {
        if (measurements.reportIn == 0) { //downcounter reached zero, we are publishing to mqtt
            MQTTConect();
            Serial.print(F("Sending measurements, temp: "));
            Serial.print(measurements.temperature);
            Serial.print(F(", hum: "));
            Serial.print(measurements.humidity);
            Serial.print(F(", vcc: "));
            Serial.print(measurements.voltage);
            Serial.print(F(", vcc raw: "));
            Serial.print(measurements.voltageRaw);
            Serial.print(F("... "));
            bool succ = mqttTempFeed.publish(measurements.temperature);
            succ = mqttHumFeed.publish(measurements.humidity) && succ;
            succ = mqttVccFeed.publish(measurements.voltage) && succ;
            succ = mqttVccRawFeed.publish(measurements.voltageRaw) && succ;
            //success, we will reset counter, failur wont'
            if (succ) {
                measurements.reportIn = publishEveryNMeasurements - 1;
                Serial.println(F("OK!"));
            } else {
                Serial.println(F("Failed"));
            }
            MQTTDisconnect();
        } else {
            //count down
            measurements.reportIn--;
        }

        if (!displayingTemp) {
            lcdReset();
            displayingTemp = true;
            lcd.setCursor(0,0);
            lcd.print(F("Temp: "));
            lcd.setCursor(0,1);
            lcd.print(F("Hum: "));
        }
        lcd.setCursor(10,0);
        lcd.printf_P(PSTR("%2.1f \337"), measurements.temperature);
        lcd.setCursor(10,1);
        lcd.printf_P(PSTR("%2.1f %%"), measurements.humidity);
    }
    
    //deep sleep stuff, not used yet
//     uint32_t* persistent = (uint32_t*)&measurements;
// //    char* my_s_bytes = reinterpret_cast<char*>(&my_s);
//     //ESP.rtcUserMemoryWrite(0, persistent, sizeof(persistent));
//     uint32_t zero = 0;
//     ESP.rtcUserMemoryWrite(0, &zero, sizeof(uint32_t));
//     ESP.rtcUserMemoryWrite(4, &measurements.reportIn, sizeof(uint32_t));
//     ESP.deepSleep(measurmentDelayMs * 1000, RF_NO_CAL);
    coldStart = false; //coldStart = firstRun
    delay(measurmentDelayMs); 
}

byte measureTemp() {
    unsigned int data[6];

    Wire.beginTransmission(tempSensAddr);
    // measurement command
    Wire.write(0x2C);
    Wire.write(0x06);
    if (Wire.endTransmission() != 0)  {
        return 1;
    }
    delay(500);
    // Request 6 bytes of data
    Wire.requestFrom(tempSensAddr, 6);
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


void onDisplayButtonTriggered() {
    if (!displayBacklightOn) {
        lcdTurnOnBacklight();
    }
}


void lcdTurnOffBacklight() {
    if (!displayBacklightOn) {
        return;
    }
    lcd.noBacklight();
    displayBacklightOn = false;
    displayBacklightOnSince = 0;
    displayBacklightTicker.detach();

}


void lcdTurnOnBacklight() {
    if (displayBacklightOn) {
        return;
    }
    lcd.backlight();
    displayBacklightOn = true;
    displayBacklightOnSince = millis();
    displayBacklightTicker.attach(displayBacklightOnDelayS, lcdTurnOffBacklight);
}

void MQTTConect() {
    int wifiStatus = WiFi.status();
    if(wifiStatus != WL_CONNECTED){
        Serial.println();
        Serial.print(F("WiFi not connected, status: "));
        Serial.print(wifiStatus);
        Serial.println();
        WIFIshowConnecting();
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
}

void lcdReset() {
    lcd.clear();
    lcd.home();
    displayingTemp = false;
}

void WIFIshowConnecting() {
    Serial.print(FPSTR(msgWifiConnecting));
    Serial.println(ssid);
    lcdReset();
    lcd.print(FPSTR(msgWifiConnecting));
    lcd.setCursor(0,1);
    lcd.print(ssid);
    if (coldStart) {
        lcdTurnOnBacklight();
    }
    
}

void WIFIShowConnected() {
    Serial.print(F("Your ESP is connected! Your IP address is: "));  
    Serial.println(WiFi.localIP());      
    lcdReset();
    lcd.print(F("IP Address:"));
    lcd.setCursor(0,1);
    lcd.print(WiFi.localIP());
    if (coldStart) {
        lcdTurnOnBacklight();
    }
    
    delay(1000);
    lcdReset();
}

void WIFIConect(bool debugBlink) {
    WIFIshowConnecting();    
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        if (coldStart) {
            lcdTurnOnBacklight();
        }
        
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
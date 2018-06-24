#include <Arduino.h>
#include "ESP8266WiFi.h"
#include "Wire.h"
#include "keys.h" //this file is not versioned and should contain only ssid and password 
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <LiquidCrystal_I2C.h>

ADC_MODE(ADC_VCC);

const char aioSslFingreprint[] = "77 00 54 2D DA E7 D8 03 27 31 23 99 EB 27 DB CB A5 4C 57 18";

const byte tempSensAddr = 0x45; 
const byte displayAddr = 0x27;
const byte ledPin = D0;
const char aioServer[] = "io.adafruit.com";
//const char aioServer[] = "192.168.178.29";
const int aioServerport = 8883; //ssl 8883, no ssl 1883;

const char ssid[] = MYSSID; //put #define MYSSID "xyz" in keys.h
const char password[] = MYPASS; //put #define MYPASS "blf" in keys.h
const char aioUsername[] = AIO_USERNAME; //put #define AIO_USERNAME "xyz" in keys.h
const char aioKey[] = AIO_KEY; //put #define AIO_KEY "xyz" in keys.h
const char tempfeed[] = AIO_USERNAME "/feeds/room-monitor.temperature";
const char humfeed[] = AIO_USERNAME "/feeds/room-monitor.humidity";
const char battery[] = AIO_USERNAME "/feeds/room-monitor.battery";

const char msgWifiConnecting[] PROGMEM = "WIFI connecting to: ";

struct Measurements {
    float temperature = 0.0;
    float humidity = 0.0;
    float voltage = 0.0;
} measurements;


LiquidCrystal_I2C lcd(displayAddr, 16, 2);

WiFiClientSecure client;
//WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, aioServer, aioServerport, aioUsername, aioKey);
Adafruit_MQTT_Publish mqttTempFeed = Adafruit_MQTT_Publish(&mqtt,tempfeed, MQTT_QOS_0);
Adafruit_MQTT_Publish mqttHumFeed = Adafruit_MQTT_Publish(&mqtt,humfeed, MQTT_QOS_0);


void MQTT_connect();
void WIFI_connect(bool debugBlink);
void verifyFingerprint();
byte measureTemp();
void lcdReset();

void setup() {
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
    WiFi.mode(WIFI_STA);
    Wire.begin(/*sda*/D2, /*scl*/D1);
    long deepSleepMax = ESP.deepSleepMax();
    lcd.init();
    lcd.backlight();
    lcdReset();
    lcd.home();
    lcd.print("Starting...");
    Serial.printf_P(PSTR("Starting... deep sleep max: %d"), deepSleepMax);
    Serial.println();
    WIFI_connect(true);
    verifyFingerprint();
}

int x = 0;
bool displayingTemp = false;
void loop() {
    MQTT_connect();
    measurements.voltage = ESP.getVcc() / 1024.00f;
    
    
    byte measureRes = measureTemp(); 
    if (measureRes != 0) {
        Serial.println(F("Unable to measure temperature"));
        Serial.println(measureRes);
    } else {
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

        Serial.print(F("Sending measurements, temp: "));
        Serial.print(measurements.temperature);
        Serial.print(F(", hum: "));
        Serial.print(measurements.humidity);
        Serial.print(F(", vcc: "));
        Serial.print(measurements.voltage);
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
    delay(10000); 
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

void lcdReset() {
    lcd.clear();
    displayingTemp = false;
}

void WIFI_connect(bool debugBlink) {
    Serial.print(FPSTR(msgWifiConnecting));
    Serial.println(ssid);

    lcdReset();
    lcd.home();
    lcd.print(FPSTR(msgWifiConnecting));
    lcd.setCursor(0,1);
    lcd.print(ssid);
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

    lcd.clear();
    lcd.home();
    lcd.print(F("IP Address:"));
    lcd.setCursor(0,1);
    lcd.print(WiFi.localIP());
    delay(1000);
    lcd.clear();
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

}
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include "keys.h" //this file is not versioned and should contain only ssid and password 
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal_I2C.h>
#include <Ticker.h>

extern "C" {
// #include "c_types.h"
// #include "ets_sys.h"
// #include "os_type.h"
// #include "osapi.h"
// #include "mem.h"
#include "gpio.h"
#include "user_interface.h"

// #include "lwip/opt.h"
// #include "lwip/err.h"
// #include "lwip/dns.h"
// #include "lwip/init.h" // LWIP_VERSION_
}

const char aioSslFingreprint[] = "77 00 54 2D DA E7 D8 03 27 31 23 99 EB 27 DB CB A5 4C 57 18";
const int powerLowerThanWarningThresholds[] = {490, 480, 470}; //round(vcc * 10)
const byte powerLowerThanWarningThresholdCount = 3;
// const int powerLowerThanWarningThresholds[] = {520, 510, 500}; //round(vcc * 10)

const byte tempSensAddr = 0x45; 
const byte displayAddr = 0x27;
const byte displayOnButtonPin = D3;
const byte controllAnalogIn = D5; //not: D4=GPIO2=BLUELED
//1min
const unsigned long measurmentDelayMs = 10 * 1000; //10s
const int publishEveryNMeasurements = 6; //how often will be measured value reported in relatino to measurementDelay
//5min
// const unsigned long measurmentDelayMs = 1 * 60 * 1000; //1m
// const int publishEveryNMeasurements = 5; //how often will be measured value reported in relatino to measurementDelay
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
const char vccwarning[] = AIO_USERNAME "/feeds/room-monitor.vcc-warning";
const char photorawfeed[] = AIO_USERNAME "/feeds/room-monitor.photo-raw";
const char photovfeed[] = AIO_USERNAME "/feeds/room-monitor.photo-v";
const char photorfeed[] = AIO_USERNAME "/feeds/room-monitor.photo-r";
const char pressurefeed[] = AIO_USERNAME "/feeds/room-monitor.pressure";


const char msgWifiConnecting[] PROGMEM = "WIFI connecting to: ";

struct Measurements {
    float temperature = 0.0;
    float humidity = 0.0;
    float voltage = 0.0;
    float voltageSum = 0.0;
    unsigned char voltageCount = 0;
    int voltageRaw = 0;
    int photoRaw = 0;
    float photoVoltage = 0.0;
    float photoValue = 0.0;
    int lastPowerWarningThreshold = 0;
    unsigned char reportIn = 0;
    float pressure = 0.0;
    float bmpTemp = 0.0;
} measurements;

bool displayingTemp = false;

LiquidCrystal_I2C lcd(displayAddr, 16, 2);
Adafruit_BMP280 bmp; 

WiFiClientSecure client;
//WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, aioServer, aioServerport, aioUsername, aioKey);
Adafruit_MQTT_Publish mqttTempFeed = Adafruit_MQTT_Publish(&mqtt,tempfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttHumFeed = Adafruit_MQTT_Publish(&mqtt,humfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttVccFeed = Adafruit_MQTT_Publish(&mqtt,vccfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttVccRawFeed = Adafruit_MQTT_Publish(&mqtt,vccrawfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttVccWarning = Adafruit_MQTT_Publish(&mqtt,vccwarning, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttPhotoRawFeed = Adafruit_MQTT_Publish(&mqtt,photorawfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttPhotoVFeed = Adafruit_MQTT_Publish(&mqtt,photovfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttPhotoRFeed = Adafruit_MQTT_Publish(&mqtt,photorfeed, MQTT_QOS_1);
Adafruit_MQTT_Publish mqttPressureFeed = Adafruit_MQTT_Publish(&mqtt,pressurefeed, MQTT_QOS_1);


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
    gpio_init();
    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
    pinMode(displayOnButtonPin, INPUT_PULLUP);
    pinMode(controllAnalogIn, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(displayOnButtonPin), onDisplayButtonTriggered, FALLING);
    Wire.begin(/*sda*/D2, /*scl*/D1);
    bool bmpStatus = bmp.begin(0x76);
    if (!bmpStatus) {
        Serial.println("Unable to initialize bmp280");
        while(1);
    }
    lcd.init();
    if (coldStart) {
        lcdReset();
        lcd.home();
        lcd.print(F("Starting..."));
        lcdTurnOnBacklight();
    }
    Serial.println(F("Starting..."));
    WiFi.persistent(false);
}


void printMeasurementsToSerial() {
    Serial.print(F("temp: "));
    Serial.print(measurements.temperature);
    Serial.print(F(" ("));
    Serial.print(measurements.bmpTemp);
    Serial.print(F(") "));
    Serial.print(F(", hum: "));
    Serial.print(measurements.humidity);
    Serial.print(F(", press: "));
    Serial.print(measurements.pressure);
    Serial.print(F(", vcc: "));
    Serial.print(measurements.voltage);
    Serial.print(F(", avg vcc: "));
    Serial.print(measurements.voltageSum / measurements.voltageCount);
    Serial.print(F(", vcc raw: "));
    Serial.print(measurements.voltageRaw);
    Serial.print(F(", phot R: "));
    Serial.print(measurements.photoValue);
    Serial.print(F(", phot vol: "));
    Serial.print(measurements.photoVoltage);
    Serial.print(F(", phot raw: "));
    Serial.print(measurements.photoRaw);
}

float analogToVoltage(const int analog) {
    //not used (analog / 1024.0f ) * 3.3f; //0-3.3, there's internal voltage divider 100k/220k
    //0-6.6v, external voltage divider 220/(680 + internal divider in paralel) 
    //return (analog / 1024.0f ) * 2.81f * 2.266f;
    return analog * 0.00611; //calibrated from measurements and less errors from floating point arithmetics
}

void on_wakeup() {
    Serial.println(F("Woke up..."));
    Serial.println(millis());
    wifi_fpm_close();
    wifi_set_opmode(STATION_MODE);

}

void loop() {
    // MQTTConect();
    // delay(100); //Let things settle a bit (mainly the power source voltage)
    //measure battery voltage
    digitalWrite(controllAnalogIn, LOW);
    measurements.voltageRaw = analogRead(A0);
    measurements.voltage = analogToVoltage(measurements.voltageRaw);
    measurements.voltageSum += measurements.voltage;
    measurements.voltageCount++;
    //measure light sensor
    digitalWrite(controllAnalogIn, HIGH);
    measurements.photoRaw = analogRead(A0);
    measurements.photoVoltage = analogToVoltage(measurements.photoRaw);
    //following should be the resistance of the photoresistor
    measurements.photoValue = (measurements.voltage - measurements.photoVoltage) * 10.0 / measurements.photoVoltage; /*  * 10komh */ 
    digitalWrite(controllAnalogIn, LOW);
    //SHT-30 measure temp and humidity
    byte measureRes = measureTemp(); 
    //measur pressure
    measurements.bmpTemp = bmp.readTemperature();
    measurements.pressure = bmp.readPressure();

    printMeasurementsToSerial();
    Serial.println();

    if (measureRes != 0) {
        Serial.println(F("Unable to measure temperature"));
        Serial.println(measureRes);
    } else {
        if (measurements.reportIn == 0) { //downcounter reached zero, we are publishing to mqtt
            MQTTConect();
            Serial.print(F("Sending measurements: "));
            printMeasurementsToSerial();
            Serial.print(F("... "));
            //debug - dont send vaues
            // bool succ = true;
            bool succ = mqttTempFeed.publish(measurements.temperature);
            succ = mqttHumFeed.publish(measurements.humidity) && succ;
            const float reportedVoltage = measurements.voltageSum / measurements.voltageCount;
            succ = mqttVccFeed.publish(reportedVoltage) && succ;
            measurements.voltageSum = 0.0;
            measurements.voltageCount = 0;
            succ = mqttVccRawFeed.publish(measurements.voltageRaw) && succ;
            succ = mqttPhotoRawFeed.publish(measurements.photoRaw) && succ;
            succ = mqttPhotoVFeed.publish(measurements.photoVoltage) && succ;
            succ = mqttPhotoRFeed.publish(measurements.photoValue) && succ;
            succ = mqttPressureFeed.publish(measurements.pressure) && succ;
            //Should we send power warning? We are sending value only when it goes under some threshold 
            //for the first time.
            //detect actual threshold
            const int thresholdValue = roundf(reportedVoltage * 100.0f);
            Serial.print(F("Computed threshold: "));
            Serial.print(thresholdValue);
            Serial.print(F(" last: "));
            Serial.println(measurements.lastPowerWarningThreshold);
            int currentThreshold = 0;
            for (unsigned char i = 0; i < powerLowerThanWarningThresholdCount; i++) {
                if (thresholdValue <= powerLowerThanWarningThresholds[i]) {
                    currentThreshold = powerLowerThanWarningThresholds[i];
                }
            }

            if (!coldStart && currentThreshold > 0 && currentThreshold != measurements.lastPowerWarningThreshold) {
                Serial.print(F("Reporting threshold: "));
                Serial.print(currentThreshold);
                mqttVccWarning.publish(currentThreshold);
                measurements.lastPowerWarningThreshold = currentThreshold;
            }

            //success, we will reset counter, failur wont'
            if (succ) {
                measurements.reportIn = publishEveryNMeasurements - 1;
                Serial.println(F(" OK!"));
            } else {
                Serial.println(F(" Failed"));
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

            // lcd.setCursor(0,0);
            // lcd.print(F("VCC-Raw: "));
            // lcd.setCursor(0,1);
            // lcd.print(F("VCC: "));
        }
        lcd.setCursor(10,0);
        lcd.printf_P(PSTR("%2.1f \337"), measurements.temperature);
        lcd.setCursor(10,1);
        lcd.printf_P(PSTR("%2.1f %%"), measurements.humidity);

        // lcd.setCursor(10,0);
        // lcd.printf_P(PSTR("%4d"), measurements.photoRaw);
        // lcd.setCursor(10,1);
        // lcd.printf_P(PSTR("%1.2f"), measurements.photoVoltage);
    }
    
    coldStart = false; //coldStart = firstRun

    //start sleep
    Serial.println(F("Try to initalize sleep..."));\
    Serial.flush();
    wifi_station_disconnect();
    wifi_set_opmode_current(NULL_MODE);
    wifi_set_opmode(NULL_MODE);
    wifi_fpm_open();
    gpio_pin_wakeup_enable(GPIO_ID_PIN(displayOnButtonPin), GPIO_PIN_INTR_LOLEVEL);
    // wifi_enable_gpio_wakeup(GPIO_ID_PIN(displayOnButtonPin), GPIO_PIN_INTR_LOLEVEL);
    wifi_fpm_set_wakeup_cb(on_wakeup);

    signed char result = wifi_fpm_do_sleep(10000000l);
    // signed char result = wifi_fpm_do_sleep(0xFFFFFFF);
    if (result != 0) {
        Serial.print(F("Sleep failed..."));
        Serial.println(result);
        delay(measurmentDelayMs); 
    } else {
        delay(10000+1);
        // Serial.print(F("Sleep ok"));
        // Serial.println(result);
        // Serial.println(millis());
        // delay(200);
        Serial.println(F("After delay"));
    }
   
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
}


void lcdTurnOnBacklight() {
    if (displayBacklightOn) {
        return;
    }
    lcd.backlight();
    displayBacklightOn = true;
    displayBacklightOnSince = millis();
    displayBacklightTicker.once(displayBacklightOnDelayS, lcdTurnOffBacklight);
}

void MQTTConect() {
    int wifiStatus = WiFi.status();
    if (wifi_get_opmode)
    if(wifiStatus != WL_CONNECTED){
        Serial.println();
        Serial.print(F("WiFi not connected, status: "));
        Serial.print(wifiStatus);
        Serial.println();
        WIFIshowConnecting();
        wifi_fpm_close();
        wifi_set_opmode(STATION_MODE);
        WiFi.mode(WIFI_STA);
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

void WIFIConect() {
    WIFIshowConnecting();    
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        if (coldStart) {
            lcdTurnOnBacklight();
        }
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
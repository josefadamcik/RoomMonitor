#ifndef DATA_REPORTER
#define DATA_REPORTER

#include "keys.h" //this file is not versioned and should contain only ssid and password 
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include "MeasurementProvider.h"

const int powerLowerThanWarningThresholds[] = {490, 480, 470}; //round(vcc * 10)
const byte powerLowerThanWarningThresholdCount = 3;
// const int powerLowerThanWarningThresholds[] = {520, 510, 500}; //round(vcc * 10)
const char aioSslFingreprint[] = "77 00 54 2D DA E7 D8 03 27 31 23 99 EB 27 DB CB A5 4C 57 18";
const bool vccReportingOn = false;



class WifiSetup {
    public:
        WifiSetup(const char wifiSsid[], const char wifiKey[]) 
            : ssid(wifiSsid), key(wifiKey) {};
        const char* ssid;
        const char* key;
};


class ServerSetup {
    public: 
        ServerSetup(const char server[], int port, const char usernme[], const char key[]) 
            : aioServer(server), aioServerPort(port), aioUsername(usernme), aioKey(key) {};
        const char* aioServer;
        const int aioServerPort;
        const char* aioUsername;
        const char* aioKey;
        
};

class FeedsSetup {
    public:
        FeedsSetup(
            const char tempfeedkey[],
            const char humfeedkey[],
            const char vccfeedkey[],
            const char vssrawfeedkey[],
            const char vccwarningfeedkey[],
            const char photovfeedkey[],
            const char pressurefeedkey[]
        ) 
        :   tempfeed(tempfeedkey),
            humfeed(humfeedkey),
            vccfeed(vccfeedkey),
            vccrawfeed(vssrawfeedkey),
            vccwarning(vccwarningfeedkey),
            photovfeed(photovfeedkey),
            pressurefeed(pressurefeedkey)
        {}
        const char* tempfeed;
        const char* humfeed;
        const char* vccfeed;
        const char* vccrawfeed;
        const char* vccwarning;
        const char* photovfeed;
        const char* pressurefeed;
};

class DataReporter {
    public:
        DataReporter(
            const WifiSetup& wifi,
            const ServerSetup& server, 
            const FeedsSetup& feed,
            int publishEveryNCalls 
        ) : wifiSetup(wifi),
            serverSetup(server),
            publishEveryNMeasurements(publishEveryNCalls),
            mqtt(&client, server.aioServer, server.aioServerPort, server.aioUsername, server.aioKey),
            mqttTempFeed(&mqtt,feed.tempfeed, MQTT_QOS_1),
            mqttHumFeed(&mqtt,feed.humfeed, MQTT_QOS_1),
            mqttVccFeed(&mqtt,feed.vccfeed, MQTT_QOS_1),
            mqttVccRawFeed(&mqtt,feed.vccrawfeed, MQTT_QOS_1),
            mqttVccWarning(&mqtt,feed.vccwarning, MQTT_QOS_1),
            mqttPhotoVFeed(&mqtt,feed.photovfeed, MQTT_QOS_1),
            mqttPressureFeed(&mqtt,feed.pressurefeed, MQTT_QOS_1)
        {};
        void doReport(const MeasurementsData& data);
    private:
        const WifiSetup wifiSetup;
        const ServerSetup serverSetup;
        const int publishEveryNMeasurements = 6; //how often will be measured value reported in relatino to measurementDelay        
        WiFiClientSecure client;
        Adafruit_MQTT_Client mqtt;
        Adafruit_MQTT_Publish mqttTempFeed;
        Adafruit_MQTT_Publish mqttHumFeed;
        Adafruit_MQTT_Publish mqttVccFeed;
        Adafruit_MQTT_Publish mqttVccRawFeed;
        Adafruit_MQTT_Publish mqttVccWarning;
        Adafruit_MQTT_Publish mqttPhotoVFeed;
        Adafruit_MQTT_Publish mqttPressureFeed;

        int reportIn = 0;
        int lastPowerWarningThreshold = 0;

        void MQTTConect();
        void MQTTDisconnect();
        void WIFIConect(bool debugBlink);
        void WIFIshowConnecting();
        void WIFIShowConnected();
        void verifyFingerprint();
};

void DataReporter::doReport(const MeasurementsData& measurementData) {
    if (reportIn == 0) { //downcounter reached zero, we are publishing to mqtt
        MQTTConect();
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
        Serial.println(lastPowerWarningThreshold);
        int currentThreshold = 0;
        for (unsigned char i = 0; i < powerLowerThanWarningThresholdCount; i++) {
            if (thresholdValue <= powerLowerThanWarningThresholds[i]) {
                currentThreshold = powerLowerThanWarningThresholds[i];
            }
        }

        if (vccReportingOn 
                && currentThreshold > 0 
                && currentThreshold != lastPowerWarningThreshold) {
            Serial.print(F("Reporting threshold: "));
            Serial.print(currentThreshold);
            mqttVccWarning.publish(currentThreshold);
            lastPowerWarningThreshold = currentThreshold; //TODO: separate somehow
        }

        //success, we will reset counter, failur wont'
        if (succ) {
            reportIn = publishEveryNMeasurements - 1; //TODO
            Serial.println(F(" OK!"));
        } else {
            Serial.println(F(" Failed"));
        }
        MQTTDisconnect();
    } else {
        //count down
        reportIn--;
    }
}

void DataReporter::MQTTConect() {
    int wifiStatus = WiFi.status();
    if (wifiStatus != WL_CONNECTED){
        Serial.println();
        Serial.print(F("WiFi not connected, status: "));
        Serial.print(wifiStatus);
        Serial.println();
        WiFi.mode(WIFI_STA);
        //WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
        if (wifiStatus == WL_DISCONNECTED || wifiStatus == WL_CONNECTION_LOST) {
            WiFi.reconnect();
        } else {
            WiFi.begin(wifiSetup.ssid, wifiSetup.key);
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



void DataReporter::MQTTDisconnect() {
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




void DataReporter::WIFIConect(bool debugBlink) {
    WiFi.begin(wifiSetup.ssid, wifiSetup.key);
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        delay(250);
        Serial.print(".");
    }   
    Serial.println(F("Setup done"));
    verifyFingerprint();
}


void DataReporter::verifyFingerprint() {
  Serial.println(serverSetup.aioServer);
  if (! client.connect(serverSetup.aioServer, serverSetup.aioServerPort)) {
    Serial.println(F("Connection failed."));
    while(1);
  }
  if (client.verify(aioSslFingreprint, serverSetup.aioServer)) {
    Serial.println(F("Connection secure."));
  } else {
    Serial.println(F("Connection insecure!"));
    while(1);
  }

  client.stop(); //otherwise the MQTT.connected() will return true, because the implementation
  // just asks the client if there is a connectioni. It actully doesn't check if there was a mqtt connection established.

}

#endif

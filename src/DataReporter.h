#ifndef DATA_REPORTER_h
#define DATA_REPORTER_h

#include "keys.h" //this file is not versioned and should contain only ssid and password 
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include "debug.h"
#include "BatteryMonitor.h"
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
            BatteryMonitor* battery
        ) : wifiSetup(wifi),
            serverSetup(server),
            mqtt(&client, server.aioServer, server.aioServerPort, server.aioUsername, server.aioKey),
            mqttTempFeed(&mqtt,feed.tempfeed, MQTT_QOS_1),
            mqttHumFeed(&mqtt,feed.humfeed, MQTT_QOS_1),
            mqttVccFeed(&mqtt,feed.vccfeed, MQTT_QOS_1),
            mqttVccRawFeed(&mqtt,feed.vccrawfeed, MQTT_QOS_1),
            mqttVccWarning(&mqtt,feed.vccwarning, MQTT_QOS_1),
            mqttPhotoVFeed(&mqtt,feed.photovfeed, MQTT_QOS_1),
            mqttPressureFeed(&mqtt,feed.pressurefeed, MQTT_QOS_1),
            batteryMonitor(battery)
        {};
        /** Call in setup() */
        void begin();
        void ensureWifiConnection();
        void doReport(const MeasurementsData& data);
        void closeConnections();
    private:
        const WifiSetup wifiSetup;
        const ServerSetup serverSetup;
        WiFiClientSecure client;
        Adafruit_MQTT_Client mqtt;
        Adafruit_MQTT_Publish mqttTempFeed;
        Adafruit_MQTT_Publish mqttHumFeed;
        Adafruit_MQTT_Publish mqttVccFeed;
        Adafruit_MQTT_Publish mqttVccRawFeed;
        Adafruit_MQTT_Publish mqttVccWarning;
        Adafruit_MQTT_Publish mqttPhotoVFeed;
        Adafruit_MQTT_Publish mqttPressureFeed;
        BatteryMonitor* batteryMonitor;
        void MQTTConect();
        void MQTTDisconnect();
        void WIFIConect();
        void verifyFingerprint();
};

#endif

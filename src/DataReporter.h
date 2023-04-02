#ifndef DATA_REPORTER_h
#define DATA_REPORTER_h

#include "keys.h" //this file is not versioned and should contain only ssid and password 
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "debug.h"
#include "BatteryMonitor.h"
#include "MeasurementProvider.h"
#include <PubSubClient.h>

const int powerLowerThanWarningThresholds[] = {490, 480, 470}; //round(vcc * 10)
const byte powerLowerThanWarningThresholdCount = 3;
// const int powerLowerThanWarningThresholds[] = {520, 510, 500}; //round(vcc * 10)
const bool vccReportingOn = false;

class WifiSetup {
    public:
     WifiSetup(
        const char wifiSsid[], 
        const char wifiKey[], 
        IPAddress ip,
        IPAddress gateway,
        IPAddress subnet
    ) : ssid(wifiSsid), key(wifiKey), ip(ip), gateway(gateway), subnet(subnet) {};
     const char* ssid;
     const char* key;
     IPAddress ip;
     IPAddress gateway;
     IPAddress subnet;
};

class ServerSetup {
    public: 
        ServerSetup(const char server[], int port, const char login[], const char pass[]) 
            : mqttServer(server), mqttServerPort(port), mqttLogin(login), mqttPass(pass) {};
        const char* mqttServer;
        const int mqttServerPort;
        const char* mqttLogin;
        const char* mqttPass;
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
            feedsSetup(feed),
            batteryMonitor(battery)
        {};
        /** Call in setup() */
        void begin(const RoomMonitorState& state);
        void doReport(const MeasurementsData& data);
        void closeConnections();
        RoomMonitorState getState(bool baterryWarningTriggered);
    private:
        const WifiSetup wifiSetup;
        const ServerSetup serverSetup;
        const FeedsSetup feedsSetup;
        WiFiClient client;
        PubSubClient pubSubClient;
        BatteryMonitor* batteryMonitor;
        void MQTTConect();
        void MQTTDisconnect();
        void WIFIConect(const RoomMonitorState& state);
};

#endif

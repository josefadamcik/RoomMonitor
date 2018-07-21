#ifndef DATA_REPORTER
#define DATA_REPORTER

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "keys.h" //this file is not versioned and should contain only ssid and password 


class ServerSetup {
    public: 
        ServerSetup(char& server, int port) : aioServer(server), aioServerPort(port) {};
    private:
    char aioServer;
        int aioServerPort;
        
};

// class FeedsSetup {

// };

class DataReporter {
    public:
        DataReporter() {}
    private:
        // Adafruit_MQTT_Client mqtt(&client, aioServer, aioServerport, aioUsername, aioKey);
        // Adafruit_MQTT_Publish mqttTempFeed = Adafruit_MQTT_Publish(&mqtt,tempfeed, MQTT_QOS_1);
        // Adafruit_MQTT_Publish mqttHumFeed = Adafruit_MQTT_Publish(&mqtt,humfeed, MQTT_QOS_1);
        // Adafruit_MQTT_Publish mqttVccFeed = Adafruit_MQTT_Publish(&mqtt,vccfeed, MQTT_QOS_1);
        // Adafruit_MQTT_Publish mqttVccRawFeed = Adafruit_MQTT_Publish(&mqtt,vccrawfeed, MQTT_QOS_1);
        // Adafruit_MQTT_Publish mqttVccWarning = Adafruit_MQTT_Publish(&mqtt,vccwarning, MQTT_QOS_1);
        // Adafruit_MQTT_Publish mqttPhotoVFeed = Adafruit_MQTT_Publish(&mqtt,photovfeed, MQTT_QOS_1);
        // Adafruit_MQTT_Publish mqttPressureFeed = Adafruit_MQTT_Publish(&mqtt,pressurefeed, MQTT_QOS_1);
};

#endif
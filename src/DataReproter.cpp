#include "DataReporter.h"


void DataReporter::begin() {
    WiFi.persistent(false);
}

void DataReporter::ensureWifiConnection() {
    WIFIConect();
}

void DataReporter::closeConnections() {
    MQTTDisconnect();
}

void DataReporter::doReport(const MeasurementsData& measurementData) {
    MQTTConect();
    Serial.print(F("Sending measurements: "));
    measurementData.printToSerial();
    Serial.print(F("... "));
    // bool succ = true;
    bool succ = mqttTempFeed.publish(measurementData.temperature);
    succ = mqttHumFeed.publish(measurementData.humidity) && succ;
    const float reportedVoltage = measurementData.voltage;
    succ = mqttVccFeed.publish(reportedVoltage) && succ;
    succ = mqttVccRawFeed.publish(measurementData.voltageRaw) && succ;
    succ = mqttPhotoVFeed.publish(measurementData.lightLevel) && succ;
    succ = mqttPressureFeed.publish(measurementData.pressure) && succ;

    bool triggerVccWarning = batteryMonitor->checkBattery(measurementData.voltage);

    if (triggerVccWarning) {
        Serial.println(F("Triggered battery monitor"));
        succ = mqttVccWarning.publish(measurementData.voltage) && succ;
    }

    //success, we will reset counter, failur wont'
    if (succ) {
        Serial.println(F(" OK!"));
    } else {
        Serial.println(F(" Failed"));
    }
}

void DataReporter::MQTTConect() {
    int wifiStatus = WiFi.status();
    if (wifiStatus != WL_CONNECTED){
        Serial.println();
        Serial.print(F("WiFi not connected, status: "));
        Serial.print(wifiStatus);
        Serial.println();
        if (WiFi.getMode() != WIFI_STA) {
            WiFi.mode(WIFI_STA);
        }
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
        Serial.println(F("Retr MQTT connection in 1 second..."));
        mqtt.disconnect();
        delay(1000); 
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




void DataReporter::WIFIConect() {
    WiFi.begin(wifiSetup.ssid, wifiSetup.key);
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print(F("."));
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

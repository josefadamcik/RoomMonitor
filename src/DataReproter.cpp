#include "DataReporter.h"

void DataReporter::begin(const RoomMonitorState& state) {
    WiFi.forceSleepWake();
    delay(1);
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.config(wifiSetup.ip, wifiSetup.gateway, wifiSetup.subnet, wifiSetup.gateway, wifiSetup.gateway);
    WIFIConect(state);
}

RoomMonitorState DataReporter::getState(bool baterryWarningTriggered) {
    RoomMonitorState newState;
    newState.triggered = baterryWarningTriggered;
    if (WiFi.status() == WL_CONNECTED) {
        newState.channel = WiFi.channel();
        memcpy(newState.bssid, WiFi.BSSID(), 6);
        newState.validWifi = true;
    } else {
        newState.validWifi = false;
    }
    return newState;
}

void DataReporter::closeConnections() {
    MQTTDisconnect();
}

void DataReporter::doReport(const MeasurementsData& measurementData) {
    MQTTConect();
    Serial.print(F("Sending measurements: "));
    measurementData.printToSerial();
    Serial.print(F("... "));
    bool succ = mqttTempFeed.publish(measurementData.temperature);
    succ = mqttHumFeed.publish(measurementData.humidity) && succ;
    succ = mqttVccFeed.publish(measurementData.voltage) && succ;
    succ = mqttVccRawFeed.publish(measurementData.voltageRaw) && succ;
    succ = mqttPhotoVFeed.publish(measurementData.lightLevel) && succ;
    succ = mqttPressureFeed.publish(measurementData.pressure) && succ;

    bool triggerVccWarning = batteryMonitor->checkBattery(measurementData.voltage);

    if (triggerVccWarning) {
        Serial.println(F("Triggered battery monitor"));
        succ = mqttVccWarning.publish(measurementData.voltage) && succ;
    }

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
        return;
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
        delay(250); 
        retries--;
        if (retries == 0) {
            return; //lets go to sleep we'll see the next time..
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
        WiFi.disconnect(true);
        delay(1);
        Serial.println("Wifi disconnected...");
    }
}

void DataReporter::WIFIConect(const RoomMonitorState& state) {
    bool quickSetup = false;
    if (state.valid && state.validWifi) {
        quickSetup = true;
        WiFi.begin(wifiSetup.ssid, wifiSetup.key, state.channel, state.bssid, true);
    } else {
        WiFi.begin(wifiSetup.ssid, wifiSetup.key);
    }
    int retries = 0;
    int wifiStatus = WiFi.status();
    while (wifiStatus != WL_CONNECTED) {
        retries++;
        if (retries == 100 && quickSetup) {
            Serial.println(F("QC Failed"));
            // Quick connect is not working, reset WiFi and try regular
            // connection
            WiFi.disconnect();
            delay(10);
            WiFi.forceSleepBegin();
            delay(10);
            WiFi.forceSleepWake();
            delay(10);
            WiFi.begin(wifiSetup.ssid, wifiSetup.key);
        }
        if (retries == 600) {
            Serial.println(F("Give up"));
            // Giving up after 30 seconds and going back to sleep
            WiFi.disconnect(true);
            delay(1);
            WiFi.mode(WIFI_OFF);
            return;
        }
        if (retries % 10 == 0) {
            Serial.print(F("."));
        }
        delay(50);
        wifiStatus = WiFi.status();
    }
    Serial.println(WiFi.status());
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

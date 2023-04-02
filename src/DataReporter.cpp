#include "DataReporter.h"

void DataReporter::begin(const RoomMonitorState& state) {
    WiFi.forceSleepWake();
    delay(1);
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.config(wifiSetup.ip, wifiSetup.gateway, wifiSetup.subnet, wifiSetup.gateway, wifiSetup.gateway);
    WIFIConect(state);

    pubSubClient.setClient(client);
    pubSubClient.setServer(serverSetup.mqttServer, serverSetup.mqttServerPort);
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
    char tmpStr[8]; // Buffer big enough for 7-character float
    dtostrf(measurementData.temperature, 4, 2, tmpStr);
    bool succ = pubSubClient.publish(feedsSetup.tempfeed, tmpStr, true);
    dtostrf(measurementData.humidity, 4, 2, tmpStr);
    succ = succ && pubSubClient.publish(feedsSetup.humfeed, tmpStr, true);
    dtostrf(measurementData.voltage, 4, 2, tmpStr);
    succ = succ && pubSubClient.publish(feedsSetup.vccfeed, tmpStr, true);
    dtostrf(measurementData.voltageRaw, 1, 0, tmpStr);
    succ = succ && pubSubClient.publish(feedsSetup.vccrawfeed, tmpStr, true);
    dtostrf(measurementData.lightLevel, 1, 0, tmpStr);
    succ = succ && pubSubClient.publish(feedsSetup.photovfeed, tmpStr, true);
    dtostrf(measurementData.pressure, 1, 0, tmpStr);
    succ = succ && pubSubClient.publish(feedsSetup.pressurefeed, tmpStr, true);

    bool triggerVccWarning = batteryMonitor->checkBattery(measurementData.voltage);

    if (triggerVccWarning) {
        Serial.println(F("Triggered battery monitor"));
        dtostrf(measurementData.voltage, 6, 2, tmpStr);
        succ = succ && pubSubClient.publish(feedsSetup.vccwarning, tmpStr, true);
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

    // Stop if already connected, but double check with ping
    if (pubSubClient.connected()) {
        return;
    }
    Serial.print(F("Connecting to MQTT... "));

    int8_t ret = 0;
    uint8_t retries = 3;
    while (!pubSubClient.connect("RoomMonitor." ROOM_NAME, serverSetup.mqttLogin, serverSetup.mqttPass)) { // connect will return 0 for connected
        ret = pubSubClient.state();
        Serial.println(F("Retr MQTT connection in 1 second..."));
        Serial.println(ret);
        pubSubClient.disconnect();
        delay(250); 
        retries--;
        if (retries == 0) {
            return; //lets go to sleep we'll see the next time..
        }
    }

    if (pubSubClient.connected()) {
        Serial.println(F(" MQTT Connected!"));
    } else {
        Serial.print(F(" MQTT still NOT onnected! "));
        Serial.println(ret);
    }
}

void DataReporter::MQTTDisconnect() {
    if (pubSubClient.connected()) {
        pubSubClient.disconnect();
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
}


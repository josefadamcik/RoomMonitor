#ifndef MEASUREMENTS_PROVIDER_h
#define MEASUREMENTS_PROVIDER_h

#include <Arduino.h>
#include "config.h"
#include <BH1750.h>
#ifdef USE_BMP280
#include <Adafruit_BMP280.h>
#endif
#include <Adafruit_Sensor.h>
#include "RoomMonitorState.h"

class MeasurementsData {
    public:
        float temperature = 0.0;
        float humidity = 0.0;
        float voltage = 0.0;
        int voltageRaw = 0;
        unsigned char reportIn = 0;
        float pressure = 0.0;
        float bmpTemp = 0.0;
        int lightLevel = 0;
        void printToSerial() const;
    private:
};

class MeasurementProvider {
    public:
        MeasurementProvider(uint8_t tempSensAddr,  uint8_t lightSensAddr, float analogVCCToRealCoeficient);
        const MeasurementsData& getCurrentMeasurements();
        /** @return true for success. */
        bool begin();
        /** @return true for success. */
        bool doMeasurements();
        RoomMonitorState getWifiSetup();
    private : 
        const uint8_t tempSensAddress;
        BH1750 lightSensor;
        const float analogVCCToRealCoeficient;
        #ifdef USE_BMP280
        Adafruit_BMP280 bmp; 
        #endif
        MeasurementsData data;
        #ifdef USE_SHT30
        uint8_t measureTempSTH30();
        #endif
        #ifdef USE_SHT21
        uint8_t measureTempSTH21(); 
        float readFloatSHT21(uint8_t command);
        #endif
        float analogToVoltage(int analog);
};

#endif 


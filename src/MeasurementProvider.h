#ifndef MEASUREMENTS_PROVIDER
#define MEASUREMENTS_PROVIDER

#include <Arduino.h>
#include <BH1750.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>


class MeasurementsData {
    public:
        float temperature = 0.0;
        float humidity = 0.0;
        float voltage = 0.0;
        float voltageSum = 0.0;
        unsigned char voltageCount = 0;
        int voltageRaw = 0;
        int lastPowerWarningThreshold = 0;
        unsigned char reportIn = 0;
        float pressure = 0.0;
        float bmpTemp = 0.0;
        int lightLevel = 0;
        void printToSerial();
    private:
};

class MeasurementProvider {
    public:
        MeasurementProvider(uint8_t tempSensAddr,  uint8_t lightSensAddr);
        MeasurementsData getCurrentMeasurements();
        /** @return true for success. */
        bool begin();
        /** @return true for success. */
        bool doMeasurements();
    private:
        uint8_t tempSensAddress;
        Adafruit_BMP280 bmp; 
        BH1750 lightSensor;
        MeasurementsData data;

        uint8_t measureTemp();
        float analogToVoltage(int analog);
};

MeasurementProvider::MeasurementProvider(uint8_t tempSensAddr,  uint8_t lightSensAddr) 
    : tempSensAddress(tempSensAddr), lightSensor(lightSensAddr)  {
}







#endif 


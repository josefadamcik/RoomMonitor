#include "MeasurementProvider.h"

#define SHT21_TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
#define SHT21_TRIGGER_HUMD_MEASURE_NOHOLD  0xF5

MeasurementProvider::MeasurementProvider(uint8_t tempSensAddr,  uint8_t lightSensAddr) 
     : tempSensAddress(tempSensAddr), lightSensor(lightSensAddr)  {
}

bool MeasurementProvider::begin() {
    bool lsStatus = lightSensor.begin(BH1750::ONE_TIME_HIGH_RES_MODE); //goes to sleep after mea
    // bool bmpStatus = bmp.begin(0x76);
    // return true;
    return lsStatus;// && bmpStatus;
}

const MeasurementsData& MeasurementProvider::getCurrentMeasurements(){
    return data;
}

bool MeasurementProvider::doMeasurements() {
    //measure battery voltage
    data.voltageRaw = analogRead(A0);
    data.voltage = analogToVoltage(data.voltageRaw);
    //SHT-30 measure temp and humidity
    // byte tempMeasureRes;
    // int retryMeasurement = 3;
    // do {
    //     retryMeasurement--;
    //     tempMeasureRes = measureTempSTH30();
    //     if (tempMeasureRes != 0) {
    //         Serial.println(F("Unable to measure temperature"));
    //         Serial.println(tempMeasureRes);
    //         delay(100);
    //     }
    // } while (tempMeasureRes != 0 && retryMeasurement > 0);

    // if (tempMeasureRes != 0) {
    //     return false;
    // }
    //SHT21
    data.temperature = 0;
    data.humidity = 0;
    // measureTempSTH21();

    // measur pressure
    // data.bmpTemp = bmp.readTemperature();
    // data.pressure = bmp.readPressure();
    data.bmpTemp = 0;
    data.pressure = 0;
    // measure lipht
    data.lightLevel = lightSensor.readLightLevel();

    return true;
}

byte MeasurementProvider::measureTempSTH30() {
    unsigned int data[6];
    Wire.beginTransmission(tempSensAddress);
    // measurement command -> one shot measurement, clock stretching, high repeatability
    Wire.write(0x2C); 
    Wire.write(0x06);
    uint8_t wireResult = Wire.endTransmission();
    if (wireResult != 0)  {
        return wireResult;
    }
    delay(500);
    // Request 6 bytes of data
    Wire.requestFrom(tempSensAddress, 6);
    // cTemp msb, cTemp lsb, cTemp crc, humidity msb, humidity lsb, humidity crc
    for (int i=0;i<6;i++) {
        data[i]=Wire.read();
    };
    delay(50);

    if (Wire.available() != 0) {
        return 20;
    }
    // Convert the data
    this->data.temperature = ((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45;
    this->data.humidity = ((((data[3] * 256.0) + data[4]) * 100) / 65535.0);

    return 0;
}

uint8_t MeasurementProvider::measureTempSTH21() {
    // Convert the data
    this->data.temperature = (-6.0 + 125.0 / 65536.0 * readFloatSHT21(SHT21_TRIGGER_TEMP_MEASURE_NOHOLD));
    this->data.humidity = (-46.85 + 175.72 / 65536.0 * readFloatSHT21(SHT21_TRIGGER_HUMD_MEASURE_NOHOLD));
    return 0;
}

float MeasurementProvider::readFloatSHT21(uint8_t command)
{
    uint16_t result;

    Wire.beginTransmission(tempSensAddress);
    Wire.write(command);
    Wire.endTransmission();
	delay(100);

    Wire.requestFrom(tempSensAddress, 3);
    while(Wire.available() < 3) {
      delay(10);
    }

    // return result
    result = ((Wire.read()) << 8);
    result += Wire.read();
	result &= ~0x0003;   // clear two low bits (status bits)
    return (float)result;
}

float MeasurementProvider::analogToVoltage(int analog) {
    //not used (analog / 1024.0f ) * 3.3f; //0-3.3, there's internal voltage divider 100k/220k
    //0-6.6v, external voltage divider 220/(680 + internal divider in paralel) 
    //return (analog / 1024.0f ) * 2.81f * 2.266f;
    return analog * 0.004904651; //coeficient for white breakout and 1m/250k voltage divider.
    // keeficient for D1 MINI 0.00611; //calibrated from measurements and less errors from floating point arithmetics
}

void MeasurementsData::printToSerial() const {
    Serial.print(F("temp: "));
    Serial.print(temperature);
    Serial.print(F(" ("));
    Serial.print(bmpTemp);
    Serial.print(F(") "));
    Serial.print(F(", hum: "));
    Serial.print(humidity);
    Serial.print(F(", press: "));
    Serial.print(pressure);
    Serial.print(F(", vcc: "));
    Serial.print(voltage);
    // Serial.print(F(", avg vcc: "));
    // Serial.print(voltageSum / voltageCount);
    Serial.print(F(", vcc raw: "));
    Serial.print(voltageRaw);
    Serial.print(F(", light: "));
    Serial.print(lightLevel);
}

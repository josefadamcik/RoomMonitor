#include "MeasurementProvider.h"

MeasurementProvider::MeasurementProvider(uint8_t tempSensAddr,  uint8_t lightSensAddr) 
     : tempSensAddress(tempSensAddr), lightSensor(lightSensAddr)  {
}


bool MeasurementProvider::begin() {
    bool lsStatus = lightSensor.begin(BH1750::ONE_TIME_HIGH_RES_MODE); //goes to sleep after mea
    bool bmpStatus = bmp.begin(0x76);
    return lsStatus && bmpStatus;
}

const MeasurementsData& MeasurementProvider::getCurrentMeasurements(){
    return data;
}

bool MeasurementProvider::doMeasurements() {
    //measure battery voltage
    data.voltageRaw = analogRead(A0);
    data.voltage = analogToVoltage(data.voltageRaw);
    // data.voltageSum += data.voltage;
    // data.voltageCount++;
    //SHT-30 measure temp and humidity
    byte tempMeasureRes;
    int retryMeasurement = 3;
    do {
        retryMeasurement--;
        tempMeasureRes = measureTemp();
        if (tempMeasureRes != 0) {
            Serial.println(F("Unable to measure temperature"));
            Serial.println(tempMeasureRes);
        }
    } while (tempMeasureRes != 0 && retryMeasurement > 0);

    if (tempMeasureRes != 0) {
        return false;
    }

    // measur pressure
    data.bmpTemp = bmp.readTemperature();
    data.pressure = bmp.readPressure();
    // measure lipht
    data.lightLevel = lightSensor.readLightLevel();

    return true;
}

byte MeasurementProvider::measureTemp() {
    unsigned int data[6];
    Wire.setTimeout(500);
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

float MeasurementProvider::analogToVoltage(int analog) {
    //not used (analog / 1024.0f ) * 3.3f; //0-3.3, there's internal voltage divider 100k/220k
    //0-6.6v, external voltage divider 220/(680 + internal divider in paralel) 
    //return (analog / 1024.0f ) * 2.81f * 2.266f;
    return analog * 0.00611; //calibrated from measurements and less errors from floating point arithmetics
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

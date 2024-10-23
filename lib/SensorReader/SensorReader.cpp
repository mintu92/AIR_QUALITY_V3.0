#include "SensorReader.h"

SensorReader::SensorReader(uint8_t rxPin, uint8_t txPin) : swSer(rxPin, txPin) {
    // Initialize sensor data values to 0
    memset(buffer, 0, sizeof(buffer));
    co2 = pm25 = pm10 = lux = ns = tvoc = mot = 0;
    o3 = tm = hum = 0;
}

void SensorReader::begin(uint32_t baudRate) {
    swSer.begin(baudRate);
}

bool SensorReader::readSensorData() {
    if (swSer.available() > 0) {
        for (int i = 0; i < 25; i++) {
            buffer[i] = swSer.read();
        }

        // Flush any remaining garbage data
        while (swSer.available() > 0) {
            swSer.read();
        }

        if (validateCRC()) {

            tvoc = (buffer[3] << 8) | buffer[4];
            pm25 = (buffer[5] << 8) | buffer[6];
            pm10 = (buffer[7] << 8) | buffer[8];
            co2 = (buffer[9] << 8) | buffer[10];

            int16_t tmp = (buffer[11] << 8) | buffer[12];
            int sign = (tmp & 0x8000) ? -1 : 1;
            int magnitude = tmp & 0x7FFF;
            tm = sign * magnitude * 0.1f;

            hum = ((buffer[13] << 8) | buffer[14]) / 10;
            o3 = (buffer[15] << 8) | buffer[16];
            lux = (buffer[17] << 8) | buffer[18];
            ns = (buffer[19] << 8) | buffer[20];
            mot = (buffer[21] << 8) | buffer[22];

            return true;
        } else {
            Serial.println("CRC check failed");
            return false;
        }
    }
    return false;
}

uint16_t SensorReader::crc16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

bool SensorReader::validateCRC() {
    uint16_t receivedCrc = buffer[23] | (buffer[24] << 8);
    uint16_t calculatedCrc = crc16(buffer, 23);
    return receivedCrc == calculatedCrc;
}

void SensorReader::printSensorData() const {
    Serial.print("TVOC: "); Serial.println(tvoc);
    Serial.print("CO2: "); Serial.println(co2);
    Serial.print("PM2.5: "); Serial.println(pm25);
    Serial.print("PM10: "); Serial.println(pm10);
    Serial.print("Temperature: "); Serial.println(tm);
    Serial.print("Humidity: "); Serial.println(hum);
    Serial.print("O3: "); Serial.println(o3);
    Serial.print("LUX: "); Serial.println(lux);
    Serial.print("Mic Level: "); Serial.println(ns * 0.1f);
    Serial.print("Motion: "); Serial.println(mot);
}

// Getters for sensor data
int16_t SensorReader::getTVOC() const { return tvoc; }
int16_t SensorReader::getCO2() const { return co2; }
int16_t SensorReader::getPM25() const { return pm25; }
int16_t SensorReader::getPM10() const { return pm10; }
float SensorReader::getTemperature() const { return tm; }
float SensorReader::getHumidity() const { return hum; }
float SensorReader::getO3() const { return o3; }
int16_t SensorReader::getLUX() const { return lux; }
float SensorReader::getMicLevel() const { return ns * 0.1f; }
int16_t SensorReader::getMotion() const { return mot; }

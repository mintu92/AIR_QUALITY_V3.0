#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <Arduino.h>
#include <SoftwareSerial.h>

class SensorReader {
public:
    SensorReader(uint8_t rxPin, uint8_t txPin);
    void begin(uint32_t baudRate = 9600);
    bool readSensorData();
    void printSensorData() const;

    // Getters for sensor data
    int16_t getTVOC() const;
    int16_t getCO2() const;
    int16_t getPM25() const;
    int16_t getPM10() const;
    float getTemperature() const;
    float getHumidity() const;
    float getO3() const;
    int16_t getLUX() const;
    float getMicLevel() const;
    int16_t getMotion() const;

private:
    SoftwareSerial swSer;
    uint8_t buffer[25];
    int16_t co2, pm25, pm10, lux, ns, mot, tvoc;
    float o3, tm, hum;
    uint16_t crc16(const uint8_t *data, uint16_t length);

    bool validateCRC();
};

#endif // SENSOR_READER_H

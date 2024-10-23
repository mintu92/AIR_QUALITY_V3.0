#ifndef UARTSENSORREAD_H
#define UARTSENSORREAD_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

class UARTSensorRead {
public:
    UARTSensorRead(int selectPin1, int selectPin2);     // Constructor to initialize control pins and UART communication

    void begin(long baudRate);                          // Function to initialize the library
    void selectSensor(int sensor);

    float readCO2();    // CO2  sensor
    float readTVOC();   // TVOC sensor
    float readPM();     // PM   sensor
    float readUART();   // Generic UART sensor

        // BME680 specific functions
    void setupBME680();  // Set up the BME680 sensor
    float readBME680Temp();  // Read temperature from BME680
    float readBME680Gas();  // Read gas resistance from BME680
    float readBME680Hum();  // Read humidity from BME680

    void writeData(const String& data);

private:
    int _selectPin1;
    int _selectPin2;

    Adafruit_BME680 _bme;  // BME680 sensor object

    String readData();
};

#endif

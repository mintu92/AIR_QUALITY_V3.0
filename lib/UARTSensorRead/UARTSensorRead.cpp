#include "UARTSensorRead.h"
#define BME680_I2C_ADDR 0x76

// Constructor: Initialize select pins
UARTSensorRead::UARTSensorRead(int selectPin1, int selectPin2) {
    _selectPin1 = selectPin1;
    _selectPin2 = selectPin2;

    pinMode(_selectPin1, OUTPUT);
    pinMode(_selectPin2, OUTPUT);
}

// Begin UART communication
void UARTSensorRead::begin(long baudRate) {
    Serial1.begin(baudRate);  // Initialize UART1 with the given baud rate
}

// Select a sensor by controlling the multiplexer pins (0-3)
void UARTSensorRead::selectSensor(int sensor) {
    digitalWrite(_selectPin1, (sensor & 0x01));  // LSB
    digitalWrite(_selectPin2, (sensor & 0x02) >> 1);  // MSB
}

// Read data from MH-Z19 CO2 sensor
float UARTSensorRead::readCO2() {
    // Send command to request CO2 concentration
    Serial1.write(0xFF);
    Serial1.write(0x01);
    Serial1.write(0x86);
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x79);

    delay(100);  // Wait for the sensor to respond

    // Read the response
    String data = readData();
    if (data.length() >= 9) {
        int high = data[2];
        int low = data[3];
        int co2 = (high << 8) + low;
        return float(co2);
    }
    return 0; // 0 stands for ERROR in readings
}

// Read data from ZE40B TVOC sensor
float UARTSensorRead::readTVOC() {
    // Send command to request TVOC concentration (assumed command)
    Serial1.write(0xFF);
    Serial1.write(0x01);  // Assuming a request command

    delay(100);  // Wait for the sensor to respond

    // Read the response
    String data = readData();
    if (data.length() >= 5) {
        int tvoc = (data[3] << 8) + data[4];
        return float(tvoc);
    }
    return 0; // 0 stands for ERROR in readings
}

// Read data from FS00202 PM sensor
float UARTSensorRead::readPM() {
    // Send command to request particulate matter concentration (assumed command)
    Serial1.write(0x32);  // Example command for FS00202 sensor
    delay(100);  // Wait for the sensor to respond

    // Read the response
    String data = readData();
    if (data.length() >= 4) {
        int pm2_5 = (data[1] << 8) + data[2];
        return float(pm2_5) ;
    }
    return 0;  // 0 stands for ERROR in readings
}

// Read data from a generic UART sensor
float UARTSensorRead::readUART() {
    // Read the data from the generic UART sensor
    return 0;  // 0 stands for ERROR in readings
}

// Set up BME680 sensor
void UARTSensorRead::setupBME680() {
  // Initialize I2C communication
  Wire.begin();

  // Initialize BME680 sensor
  if (!_bme.begin(BME680_I2C_ADDR)) {
    Serial1.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
    // Configure BME680 settings
    _bme.setTemperatureOversampling(BME680_OS_8X);
    _bme.setHumidityOversampling(BME680_OS_2X);
    _bme.setPressureOversampling(BME680_OS_4X);
    _bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    _bme.setGasHeater(320, 150);  // 320°C for 150 ms
}

// Read temperature from BME680
float UARTSensorRead::readBME680Temp() {
    if (_bme.performReading()) {
        float tmp = _bme.temperature;
    }
    return 0;
}

// Read gas resistance from BME680
float UARTSensorRead::readBME680Gas() {
    if (_bme.performReading()) {
        float humy = _bme.gas_resistance ;
    }
    return 0;
}

// Read humidity from BME680
float UARTSensorRead::readBME680Hum() {
    if (_bme.performReading()) {
        float gas = _bme.humidity ;
    }
    return 0;
}


// Helper function to read data from the selected sensor
String UARTSensorRead::readData() {
    String data = "";
    while (Serial1.available()) {
        char c = Serial1.read();
        data += c;
    }
    return data;
}

// Write data to the selected sensor
void UARTSensorRead::writeData(const String& data) {
    Serial1.print(data);
}

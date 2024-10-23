#include <UARTSensorRead.h>

// Define the control pins (S1 and S2 control both RX and TX channels)
#define SELECT_PIN1 PA0  // Common pin for S1A and S1B
#define SELECT_PIN2 PA1  // Common pin for S2A and S2B

// Create a UARTSensorRead object
UARTSensorRead sensorController(SELECT_PIN1, SELECT_PIN2);

void setup() {
    // Start serial communication for debugging
    Serial.begin(115200);

    // Initialize the sensor controller at 9600 baud
    sensorController.begin(9600);
}

void loop() {
    // Read data from MH-Z19 CO2 sensor
    sensorController.selectSensor(0);  // Select MH-Z19 sensor
    float co2Data = sensorController.readCO2();
    Serial.println("CO2: " + String(co2Data));

    // Read data from ZE40B TVOC sensor
    sensorController.selectSensor(1);  // Select ZE40B sensor
    float tvocData = sensorController.readTVOC();
    Serial.println("TVOC: " + String(tvocData));

    // Read data from FS00202 PM sensor
    sensorController.selectSensor(2);  // Select FS00202 sensor
    float pmData = sensorController.readPM();
    Serial.println("PM: " + String(pmData));

    // Read data from a generic UART sensor
    sensorController.selectSensor(3);  // Select generic sensor
    float genericData = sensorController.readUART();
    Serial.println("Generic Sensor: " + String(genericData));

    // Wait before the next loop
    delay(2000);
}

#include <SoftwareSerial.h>
#include "UARTSensorRead.h"
#include "AQICalculator.h"

// Define the control pins (S1 and S2 control both RX and TX channels)

#define SELECT_PIN1 PA0  // Common pin for S1A and S1B
#define SELECT_PIN2 PA1  // Common pin for S2A and S2B

UARTSensorRead sensor(SELECT_PIN1, SELECT_PIN2); // Create a UARTSensorRead object
AQICalculator aqiCalculator;

void setup() 
{
    Serial.begin(9600);
    sensor.begin(9600);
    Serial.println("Sensor Reader Initialized");
}

void loop() 
{

        sensor.selectSensor(0);
        float co2 = sensor.readCO2();
        Serial.println("CO2 ->"+String(co2));

        sensor.selectSensor(1);
        float tvoc = sensor.readTVOC();
        Serial.println("TVOC ->"+String(tvoc)); 

        sensor.selectSensor(2);
        float pm = sensor.readPM();
        Serial.println("PM ->"+String(pm));

        sensor.selectSensor(3);
        float uart = sensor.readUART();
        Serial.println("UART ->"+String(uart));

        float aqi = aqiCalculator.getAQI(sensor); // Calculate AQI directly using AQICalculator library
        Serial.print("Calculated AQI: ");
        Serial.println(aqi);

    delay(2000);
}

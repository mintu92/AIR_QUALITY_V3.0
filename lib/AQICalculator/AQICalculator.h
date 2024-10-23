#ifndef AQICALCULATOR_H
#define AQICALCULATOR_H

#include <SoftwareSerial.h>
#include "UARTSensorRead.h"


class AQICalculator {
public:
    AQICalculator();
    
    int calculateCO2Score(int co2Value);      // Function to calculate CO2 score based on the value
    int calculatePM25Score(int pm25Value);
    int calculateTVOCScore(int TVOCValue);
    int calculateUARTScore(int UARTValue);
    int calculateTEMPScore(int temp);
    int calculateHUMYScore(int humy);

    float calculateAQI(int co2Score, int pm25Score,int TVOCVlaue, int UARTValue, int tmpScore, int humyScore);

    float getAQI(UARTSensorRead& sensor);     // Function to get AQI directly from SensorReader instance

private:                                       // Additional members and helper functions if needed

};

#endif

#include "AQICalculator.h"


AQICalculator::AQICalculator() {}


int AQICalculator::calculateCO2Score(int co2Value)  // Calculate CO2 score based on the provided value
{
    if (co2Value <= 400) return 9;
    else if (co2Value <= 500) return 7;
    else if (co2Value <= 600) return 5;
    else if (co2Value <= 700) return 3;
    else return 1;
}

int AQICalculator::calculatePM25Score(int pm25Value) // Calculate PM2.5 score based on the provided value
{
    if (pm25Value <= 10) return 9;
    else if (pm25Value <= 25) return 7;
    else if (pm25Value <= 50) return 5;
    else if (pm25Value <= 100) return 3;
    else return 1;
}

int AQICalculator::calculateTVOCScore(int TVOCValue) // Calculate PM2.5 score based on the provided value
{
    if (TVOCValue <= 10) return 9;
    else if (TVOCValue <= 25) return 7;
    else if (TVOCValue <= 50) return 5;
    else if (TVOCValue <= 100) return 3;
    else return 1;
}

int AQICalculator::calculateUARTScore(int UARTValue) // Calculate PM2.5 score based on the provided value
{
    if (UARTValue <= 10) return 9;
    else if (UARTValue <= 25) return 7;
    else if (UARTValue <= 50) return 5;
    else if (UARTValue <= 100) return 3;
    else return 1;
}

int AQICalculator::calculateTEMPScore(int temp) // Calculate PM2.5 score based on the provided value
{
    if (temp <= 15) return 1;
    else if (temp <= 20) return 7;
    else if (temp <= 25) return 9;
    else if (temp <= 30) return 7;
    else if (temp <= 50) return 3;
    else return 1;
}

int AQICalculator::calculateHUMYScore(int humy) // Calculate PM2.5 score based on the provided value
{
    if (humy <= 30) return 1;
    else if (humy <= 40) return 3;
    else if (humy <= 60) return 9;
    else if (humy <= 70) return 5;
    else if (humy <= 100) return 1;
    else return 1;
}


float AQICalculator::calculateAQI(int co2Score, int pm25Score, int TVOCScore, int UARTScore, int tmpScore, int humyScore) // Calculate AQI based on CO2 and PM2.5 scores
{
    int numberOfParameters = 6;
    return (co2Score + pm25Score + TVOCScore + UARTScore + tmpScore + humyScore) / (float)numberOfParameters;
}

float AQICalculator::getAQI(UARTSensorRead& sensor)             // Get AQI directly from the SensorReader instance
{
    float co2Value  = sensor.readCO2();
    float pm25Value = sensor.readPM();
    float tvocValue = sensor.readTVOC();
    float uartValue = sensor.readUART();
    float tmpValue  = sensor.readBME680Temp();
    float humValue  = sensor.readBME680Hum(); 


    
    int co2Score  = calculateCO2Score(co2Value);
    int pm25Score = calculatePM25Score(pm25Value);
    int tvocScore = calculateTVOCScore(pm25Value);
    int uartScore = calculateUARTScore(pm25Value);
    int tempScore = calculateTEMPScore(tmpValue);
    int humyScore = calculateHUMYScore(humValue);
    
    return calculateAQI(co2Score, pm25Score, tvocScore, uartScore, tempScore, humyScore);
}

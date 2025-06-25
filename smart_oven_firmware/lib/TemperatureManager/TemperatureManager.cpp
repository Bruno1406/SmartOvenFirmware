/**
 * @file TemperatureManager.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "TemperatureManager.h"
#include <math.h>

#define HEATING_HYSTERESIS 0.0f // Temperature hysteresis in Celsius
#define COOLING_HYSTERESIS 1.0f // Cooling hysteresis in Celsius

#define R0 10000
#define B 3380
#define T0 297

void TemperatureManager::init() {
    pinMode(TERMOPAR_PIN, INPUT);
    analogSetPinAttenuation(TERMOPAR_PIN, ADC_11db);  // Max input voltage ~3.9V
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH); // Relay off initially
}

void TemperatureManager::runProgram() {
    static uint32_t startTime = 0;
    static float startTemperature = 0.0f;
    if (!isRunning) {
        for (int i = 0; i < 100; ++i) {
            temperatureHistory.push(readTemperature());
        }
        startTemperature = temperatureHistory.sum()/100.0f; // Average of the first 10 readings
        startTime = millis();
        isRunning = true;
        isHeating = true; // Start heating when the program is run
    }
    currentTime = millis() - startTime;
    temperatureHistory.push(readTemperature());
    currentTemperature = temperatureHistory.sum()/100.0f;
    if (currentTime < currentCurve.rampUpDuration) {
        targetTemperature = startTemperature + 
                            (currentCurve.targetTemperature - startTemperature) * 
                            (currentTime / static_cast<float>(currentCurve.rampUpDuration));
    } else if (currentTime < currentCurve.rampUpDuration + currentCurve.holdDuration) {
        targetTemperature = currentCurve.targetTemperature;
    } else if (currentTime < currentCurve.rampUpDuration + currentCurve.holdDuration + currentCurve.coolDownDuration) {
        targetTemperature = currentCurve.targetTemperature - 
                            (currentCurve.targetTemperature - currentCurve.endTemperature) * 
                            ((currentTime - currentCurve.rampUpDuration - currentCurve.holdDuration) / static_cast<float>(currentCurve.coolDownDuration));
    } else {
        targetTemperature = currentCurve.endTemperature;
        isRunning = false; // Stop heating after the program ends
    }
    controlHeating();
    if(currentTime >= (currentCurve.rampUpDuration + currentCurve.holdDuration + currentCurve.coolDownDuration)) {
        isRunning = false; // Stop the program if the total duration is reached
    }
}

void TemperatureManager::stopProgram() {
    isRunning = false;
    digitalWrite(RELAY_PIN, HIGH); // Turn off the relay
}

float TemperatureManager::readTemperature() const {
    int rawValue = (float)analogRead(TERMOPAR_PIN);
    float voltage = (rawValue / 4095.0f) * 3.9f; // Convert ADC value to voltage
    float req= (33000 / voltage) - 10000; // Calculate resistance on the port
    float r = (10500*req) / (10500-req); // Calculate the resistance of the thermistor
    float temperature = B / log(r / (R0 * exp(-B / T0))) - 273.15f; // Convert raw ADC value to temperature in Celsius
    return temperature;
}

std::pair<float, uint32_t> TemperatureManager::getHeatingData() const {
    return {currentTemperature, currentTime};
}

float TemperatureManager::getTargetTemperature() const {
    return targetTemperature;
}

bool TemperatureManager::getFailStatus() const {
    return failStatus;
}

bool TemperatureManager::isProgramFinished() const {
    return !isRunning && currentTime >= (currentCurve.rampUpDuration + currentCurve.holdDuration + currentCurve.coolDownDuration);
}

void TemperatureManager::setTemperatureCurve(const TemperatureCurve curve) {
    currentCurve = curve;
    isRunning = false; // Reset heating state when setting a new curve
}

void TemperatureManager::controlHeating() {
    if (isHeating) {
        // Control the relay based on the current and target temperatures
        if (currentTemperature < targetTemperature + HEATING_HYSTERESIS) {
            digitalWrite(RELAY_PIN, LOW); // Turn on the relay
        } else {
            digitalWrite(RELAY_PIN, HIGH); // Turn off the relay
            isHeating = false; // Stop heating if the target temperature is reached
        }
    } else {
        if(currentTemperature < targetTemperature - COOLING_HYSTERESIS) {
            digitalWrite(RELAY_PIN, LOW); // Turn on the relay to heat up
            isHeating = true; // Start heating again
        } else {
            digitalWrite(RELAY_PIN, HIGH); // Keep the relay off
        }
    }   
}




/**
 * @file TemperatureManager.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef TEMPERATURE_MANAGER_H
#define TEMPERATURE_MANAGER_H 

#include <Arduino.h>
#include <utils.h>
#include <utility>

#define TERMOPAR_PIN 34
#define RELAY_PIN 19


class TemperatureManager {
private:
    float currentTemperature = 0.0f;
    float targetTemperature = 0.0f;
    CircularFifo<float,100> temperatureHistory;
    uint32_t currentTime = 0;
    bool failStatus = 0;
    bool isRunning = false;
    bool isHeating = false;
    TemperatureCurve currentCurve;
public:
    TemperatureManager() = default;

    void init();
    void runProgram();
    void stopProgram();
    void setTemperatureCurve(const TemperatureCurve curve);
    std::pair<float,uint32_t> getHeatingData() const;
    float getTargetTemperature() const;
    bool getFailStatus() const;
    bool isProgramFinished() const;
private:
    float readTemperature() const;
    void controlHeating();
};  


#endif // TEMPERATURE_MANAGER_H
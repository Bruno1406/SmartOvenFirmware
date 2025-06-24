/**
 * @file CommunicationManager.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

#include <utility>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <utils.h>

#define SERVICE_UUID        "0000ffe0-0000-1000-8000-00805f9b34fb"
#define OVEN_DATA_CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"
#define OVEN_PROGRAM_CHARACTERISTIC_UUID    "0000ffe2-0000-1000-8000-00805f9b34fb"
#define OVEN_STATUS_CHARACTERISTIC_UUID    "0000ffe3-0000-1000-8000-00805f9b34fb"

class CommunicationManager {
private:
    class ServerCallbacks : public BLEServerCallbacks {
    private:
        CommunicationManager* manager;
    public:
        ServerCallbacks(CommunicationManager* manager) : manager(manager) {}
        void onConnect(BLEServer *pServer) override;
        void onDisconnect(BLEServer *pServer) override;
    };

    class CharacteristicCallbacks : public BLECharacteristicCallbacks {
    private:
        CommunicationManager* manager;
    public:
        CharacteristicCallbacks(CommunicationManager* manager) : manager(manager) {}
        void onWrite(BLECharacteristic *pCharacteristic) override;
        void onRead(BLECharacteristic *pCharacteristic) override;
        void onStatus(BLECharacteristic *pCharacteristic, BLECharacteristicCallbacks::Status s, uint32_t code) override;
    };


    BLEServer *pServer = NULL;
    BLECharacteristic *pOvenDataCharacteristic = NULL;
    BLECharacteristic *pOvenProgramCharacteristic = NULL;
    BLECharacteristic *pOvenStatusCharacteristic = NULL;
    CircularFifo<std::pair<float,uint32_t>, 100> ovenDataFifo; // Circular FIFO for oven data
    CircularFifo<uint8_t, 100> ovenStatusFifo; // Circular FIFO
    TemperatureCurve currentCurve;
    bool deviceConnected = false;
    bool programReceived = false;

public:
    CommunicationManager() = default;

    void init();

    void sendOvenData();
    void sendOvenStatus();

    void pushOvenData(float temperature, uint32_t time);
    void pushOvenStatus(uint8_t status);

    bool isDeviceConnected() const;
    bool isprogramReceived() const;

    uint8_t getOvenStatus(uint8_t &status) const;
    TemperatureCurve getCurrentCurve() const;
};


#endif // COMMUNICATION_MANAGER_H
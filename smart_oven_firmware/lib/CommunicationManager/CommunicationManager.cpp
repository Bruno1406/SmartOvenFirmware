/**
 * @file CommunicationManager.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #include "CommunicationManager.h"
 #include <Arduino.h>

 void CommunicationManager::init() {
    // Create the BLE Device
    BLEDevice::init("SmartOvenESP32");

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks(this));

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pOvenDataCharacteristic = pService->createCharacteristic(
    OVEN_DATA_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_INDICATE
    );
    pOvenDataCharacteristic->addDescriptor(new BLE2902());
    pOvenDataCharacteristic->setCallbacks(new CharacteristicCallbacks(this));

    pOvenProgramCharacteristic = pService->createCharacteristic(
    OVEN_PROGRAM_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pOvenProgramCharacteristic->setCallbacks(new CharacteristicCallbacks(this));
    
    pOvenStatusCharacteristic = pService->createCharacteristic(
    OVEN_STATUS_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE
    );
    pOvenStatusCharacteristic->addDescriptor(new BLE2902());
    pOvenStatusCharacteristic->setCallbacks(new CharacteristicCallbacks(this));

    // Start the service
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
}

void CommunicationManager::sendOvenData() {
    if (pOvenDataCharacteristic) {
        if(!ovenDataFifo.isEmpty()) {
            auto [temperature, time] = ovenDataFifo.get(); 
            uint32_t temperatureInt = static_cast<uint32_t>(temperature * 100); // Convert to integer representation (e.g., 36.5C -> 3650)
            uint8_t data[8];
            data[0] = temperatureInt & 0xFF; // Low byte
            data[1] = (temperatureInt >> 8) & 0xFF; // High byte
            data[2] = (temperatureInt >> 16) & 0xFF; // Third byte
            data[3] = (temperatureInt >> 24) & 0xFF; // Fourth byte
            data[4] = time & 0xFF;
            data[5] = (time >> 8) & 0xFF; // Second byte of time
            data[6] = (time >> 16) & 0xFF; //
            data[7] = (time >> 24) & 0xFF; // Fourth byte of time
            // Set the value of the characteristic
            pOvenDataCharacteristic->setValue(data, sizeof(data));
            pOvenDataCharacteristic->indicate();
        }
    }
}

void CommunicationManager::sendOvenStatus() {
    if (pOvenStatusCharacteristic) {
        if (!ovenStatusFifo.isEmpty()) {
            uint8_t status = ovenStatusFifo.get();
            pOvenStatusCharacteristic->setValue(&status, sizeof(status));
            pOvenStatusCharacteristic->indicate();
        }
    }
}

void CommunicationManager::pushOvenData(float temperature, uint32_t time) {
    ovenDataFifo.push(std::make_pair(temperature, time));
}

void CommunicationManager::pushOvenStatus(uint8_t status) {
    ovenStatusFifo.push(status);
}

uint8_t CommunicationManager::getOvenStatus(uint8_t &status) const {
    if (pOvenStatusCharacteristic) {
        std::string value = pOvenStatusCharacteristic->getValue();
        if (value.length() > 0) {
            status = value[0]; // Assuming the status is a single byte
            return 0; // Success
        }
    }
    return 0xff; // Error: characteristic not available or no value
}

TemperatureCurve CommunicationManager::getCurrentCurve() const {
    return currentCurve; // Return the current temperature curve
}

bool CommunicationManager::isDeviceConnected() const {
    return deviceConnected;
}

bool CommunicationManager::isprogramReceived() const {
    return programReceived;
}

void CommunicationManager::ServerCallbacks::onConnect(BLEServer *pServer) {
    Serial.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
    manager->deviceConnected = true;
}

void CommunicationManager::ServerCallbacks::onDisconnect(BLEServer *pServer) {
    manager->deviceConnected = false;
    BLEDevice::startAdvertising(); // Restart advertising after disconnection
}

void CommunicationManager::CharacteristicCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    Serial.printf("Characteristic %s written\n", pCharacteristic->getUUID().toString().c_str());
    if(pCharacteristic == manager->pOvenProgramCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() >= sizeof(TemperatureCurve)) {
            // Assuming the value is a serialized TemperatureCurve
            memcpy(&manager->currentCurve, value.data(), sizeof(TemperatureCurve));
            Serial.printf("Received new temperature curve: start=%f, target=%f, end=%f, rampUp=%u, hold=%u, coolDown=%u\n",
                          manager->currentCurve.targetTemperature,
                          manager->currentCurve.endTemperature,
                          manager->currentCurve.rampUpDuration,
                          manager->currentCurve.holdDuration,
                          manager->currentCurve.coolDownDuration);
        } else {
            Serial.println("Received invalid temperature curve data.");
        }
    }
}

void CommunicationManager::CharacteristicCallbacks::onRead(BLECharacteristic *pCharacteristic) {
    Serial.printf("Characteristic %s read\n", pCharacteristic->getUUID().toString().c_str());
}

void CommunicationManager::CharacteristicCallbacks::onStatus(BLECharacteristic *pCharacteristic, BLECharacteristicCallbacks::Status s, uint32_t code) {
    Serial.printf("Characteristic %s status: %d, code: %d\n", pCharacteristic->getUUID().toString().c_str(), s, code);
   if (s != BLECharacteristicCallbacks::SUCCESS_INDICATE || s != BLECharacteristicCallbacks::SUCCESS_NOTIFY) {
        if (pCharacteristic == manager->pOvenDataCharacteristic) {
            manager->ovenDataFifo.pop(); // Remove the data from the FIFO after sending
        } else if (pCharacteristic == manager->pOvenStatusCharacteristic) {
            manager->ovenStatusFifo.pop(); // Remove the status from the FIFO after sending
        }
   }
}
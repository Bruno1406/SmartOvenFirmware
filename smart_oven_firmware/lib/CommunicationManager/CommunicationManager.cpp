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

void CommunicationManager::pushOvenStatus(OvenStatus status) {
    ovenStatusFifo.push(status);
}

OvenStatus CommunicationManager::getOvenStatus() const {
    return currentOvenStatus; // Return the current oven status
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

bool CommunicationManager::isConnectionStatusChanged() {
    bool connectionStatusChanged = this->connectionStatusChanged;
    this->connectionStatusChanged = false; // Reset the flag after checking
    return connectionStatusChanged;
}

void CommunicationManager::ServerCallbacks::onConnect(BLEServer *pServer) {
    manager->deviceConnected = true;
    manager->connectionStatusChanged = true; // Set the connection status changed flag
}

void CommunicationManager::ServerCallbacks::onDisconnect(BLEServer *pServer) {
    manager->deviceConnected = false;
    manager->connectionStatusChanged = true; // Set the connection status changed flag
    BLEDevice::startAdvertising(); // Restart advertising after disconnection
}

void CommunicationManager::CharacteristicCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    Serial.printf("Characteristic %s written\n", pCharacteristic->getUUID().toString().c_str());
    if(pCharacteristic == manager->pOvenProgramCharacteristic) {
        uint8_t* buffer = pCharacteristic->getData();
        for(int i = 0; i < pCharacteristic->getLength(); i++) {
            Serial.printf("%02X ", buffer[i]); // Print the received data in hex format
        }
        manager->currentCurve.targetTemperature = (static_cast<float>(buffer[0] | (buffer[1] << 8 | (buffer[2] << 16) | (buffer[3] << 24))) / 100.0f); // Convert to float
        manager->currentCurve.endTemperature = (static_cast<float>(buffer[4] | (buffer[5] << 8 | (buffer[6] << 16) | (buffer[7] << 24))) / 100.0f); // Convert to float
        manager->currentCurve.rampUpDuration = (buffer[8] | (buffer[9] << 8 | (buffer[10] << 16) | (buffer[11] << 24))); // Convert to uint32_t
        manager->currentCurve.holdDuration = (buffer[12] | (buffer[13] << 8 | (buffer[14] << 16) | (buffer[15] << 24))); // Convert to uint32_t
        manager->currentCurve.coolDownDuration = (buffer[16] | (buffer[17] << 8 | (buffer[18] << 16) | (buffer[19] << 24))); // Convert to uint32_t
        manager->programReceived = true;
        Serial.println(); 
    }
    if (pCharacteristic == manager->pOvenStatusCharacteristic) {
        uint8_t* value = pCharacteristic->getData();
        manager->currentOvenStatus =  static_cast<OvenStatus>(value[0]); // Assuming the first byte represents the status
        Serial.printf("Oven status updated: %d\n", manager->currentOvenStatus);
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
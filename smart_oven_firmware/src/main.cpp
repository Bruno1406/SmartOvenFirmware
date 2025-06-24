/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <Arduino.h>
#include <CommunicationManager.h>
#include <TemperatureManager.h>
#include <math.h>



#define R0 10000
#define B 3380
#define T0 297

enum State {
  WAITING_FOR_CONNECTION,
  WAITING_FOR_PROGRAM,
  WAITING_FOR_START,
  RUNNING_PROGRAM,
  PROGRAM_FINISHED,
  PROGRAM_ERROR
};

CommunicationManager commManager;
TemperatureManager tempManager;

QueueHandle_t xQueuDataComm;
QueueHandle_t xQueueStatusComm;
QueueHandle_t xQueuDataHMI;

// SemaphoreHandle_t xStateMutex;

bool test_start = false;


void taskCommunication(void *pvParameters) {
  static portTickType lastWakeTime;
  static std::pair<float, uint32_t> newovenData;
  static uint8_t newOvenStatus;
  lastWakeTime = xTaskGetTickCount();
  while (true) {
    // if (xQueueReceive(xQueuDataComm, &newovenData, 0)) {
    //   commManager.pushOvenData(newovenData.first, newovenData.second);
    // }
    // if (xQueueReceive(xQueueStatusComm, &newOvenStatus, 0)) {
    //   commManager.pushOvenStatus(newOvenStatus);
    // }
    if(test_start) {
      uint32_t time = millis();
      float temperature = 30 * sin(time); // Read the current temperature
      temperature < 0 ? temperature = -temperature : temperature; // Ensure temperature is non-negative
      commManager.pushOvenData(temperature, time); // Push the temperature data to the communication manager
      commManager.sendOvenData();  // Send oven data
      commManager.sendOvenStatus(); // Send oven status
      Serial.printf("Temperature: %.2f C, Time: %u ms\n", temperature, time);
    } else {
      Serial.println("Waiting for device connection...");
    }
    if(commManager.isDeviceConnected()) {
      test_start = true; // Set test_start to true when device is connected
    }
    
    vTaskDelayUntil(&lastWakeTime, 1000 / portTICK_PERIOD_MS); // Delay for 1 second
  }
}

// void taskTemperature(void *pvParameters) {
//   static portTickType lastWakeTime;
//   static std::pair<float, uint32_t> newOvenData;
//   static float targetTemperature = 0; // Initial target temperature
//   tempManager.setTemperatureCurve({50.0f, 20.0f, 300000, 60000, 300000}); 
//   lastWakeTime = xTaskGetTickCount();
//   while (true) {
//     if (test_start) {
//       tempManager.runProgram(); // Run the temperature program
//       newOvenData = tempManager.getHeatingData(); // Get the heating data
//       targetTemperature = tempManager.getTargetTemperature(); // Get the target temperature
//       // xQueueSend(xQueuDataComm, &newOvenData, 0); // Send the data to App
//       Serial.printf("Temperature: %.2f C, Target: %.2f, Time: %u ms\n", newOvenData.first,  targetTemperature,newOvenData.second);
//     }
//     vTaskDelayUntil(&lastWakeTime, 10 / portTICK_PERIOD_MS);
//   }
// }

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial to initialize
  Serial.println("Starting Oven work!");
  commManager.init();  // Initialize the communication manager
  tempManager.init();  // Initialize the temperature manager
  // xQueuDataComm = xQueueCreate(10, sizeof(std::pair<float, uint32_t>)); // Create a queue for oven data
  // xQueueStatusComm = xQueueCreate(10, sizeof(uint8_t)); // Create a queue for oven status
  delay(100);
  xTaskCreate(taskCommunication, "Communication", 4096, NULL, 1, NULL );
  // xTaskCreate(taskTemperature, "Temperature", 4096, NULL, 1, NULL );
}

void loop() {
//     digitalWrite(RELAY_PIN, LOW); // Turn on the relay to start heating
//     int rawValue = analogRead(TERMOPAR_PIN);
//     float voltage = (rawValue / 4095.0f) * 3.9f; // Convert ADC value to voltage
//     float req= (33000 / voltage) - 10000; // Calculate resistance on the port
//     float r = (10500*req) / (10500-req); // Calculate the resistance of the thermistor
//     float temperature = B / log(r / (R0 * exp(-B / T0))) - 273.15f; // Convert raw ADC value to temperature in Celsius
//     Serial.printf("Raw Value: %d, Voltage: %.2f V, Resistance: %.2f Ohm, Temperature: %.2f C\n", rawValue, voltage, r, temperature);
//     delay(1000); // Delay for 1 second to avoid flooding the serial output
// }
}

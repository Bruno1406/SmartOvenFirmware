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

enum FSMState {
  WAITING_FOR_CONNECTION,
  WAITING_FOR_PROGRAM,
  WAITING_FOR_START,
  RUNNING,
  FINISHED,
  ERROR,
  NUM_STATES
};

enum FSMEvent {
  CONNECTION_ESTABLISHED,
  CONNECTION_LOST,
  PROGRAM_RECEIVED,
  START_PROGRAM_SIGNAL,
  STOP_PROGRAM_SIGNAL,
  PROGRAM_FINISHED,
  PROGRAM_ERROR,
  RESET_PROGRAM,
  NUM_EVENTS,
};



FSMState transitionMatrix[NUM_STATES][NUM_EVENTS] = {
  // Current State: WAITING_FOR_CONNECTION
  {
    /* Event: CONNECTION_ESTABLISHED */ WAITING_FOR_PROGRAM,
    /* Event: CONNECTION_LOST */      WAITING_FOR_CONNECTION,
    /* Event: PROGRAM_RECEIVED */    WAITING_FOR_CONNECTION,
    /* Event: START_PROGRAM_SIGNAL */ WAITING_FOR_CONNECTION,
    /* Event: STOP_PROGRAM_SIGNAL */  WAITING_FOR_CONNECTION,
    /* Event: PROGRAM_FINISHED */    WAITING_FOR_CONNECTION,
    /* Event: PROGRAM_ERROR */       WAITING_FOR_CONNECTION,
    /* Event: RESET_PROGRAM */       WAITING_FOR_CONNECTION
  },
  // Current State: WAITING_FOR_PROGRAM
  {
    /* Event: CONNECTION_ESTABLISHED */ WAITING_FOR_PROGRAM,
    /* Event: CONNECTION_LOST */      WAITING_FOR_CONNECTION,
    /* Event: PROGRAM_RECEIVED */    WAITING_FOR_START,
    /* Event: START_PROGRAM_SIGNAL */ WAITING_FOR_PROGRAM,
    /* Event: STOP_PROGRAM_SIGNAL */  WAITING_FOR_PROGRAM,
    /* Event: PROGRAM_FINISHED */    WAITING_FOR_PROGRAM,
    /* Event: PROGRAM_ERROR */       WAITING_FOR_PROGRAM,
    /* Event: RESET_PROGRAM */       WAITING_FOR_PROGRAM
  },
  // Current State: WAITING_FOR_START
  {
    /* Event: CONNECTION_ESTABLISHED */ WAITING_FOR_START,
    /* Event: CONNECTION_LOST */      WAITING_FOR_CONNECTION,
    /* Event: PROGRAM_RECEIVED */    WAITING_FOR_START,
    /* Event: START_PROGRAM_SIGNAL */ RUNNING,
    /* Event: STOP_PROGRAM_SIGNAL */  WAITING_FOR_START,
    /* Event: PROGRAM_FINISHED */    WAITING_FOR_START,
    /* Event: PROGRAM_ERROR */       WAITING_FOR_START,
    /* Event: RESET_PROGRAM */       WAITING_FOR_START
  },
  // Current State: RUNNING
  {
    /* Event: CONNECTION_ESTABLISHED */ RUNNING,
    /* Event: CONNECTION_LOST */      RUNNING, 
    /* Event: PROGRAM_RECEIVED */    RUNNING,
    /* Event: START_PROGRAM_SIGNAL */ RUNNING,
    /* Event: STOP_PROGRAM_SIGNAL */  FINISHED,
    /* Event: PROGRAM_FINISHED */    FINISHED,
    /* Event: PROGRAM_ERROR */       ERROR,
    /* Event: RESET_PROGRAM */       RUNNING
  },
  // Current State: FINISHED
  {
    /* Event: CONNECTION_ESTABLISHED */ FINISHED,
    /* Event: CONNECTION_LOST */      WAITING_FOR_CONNECTION,
    /* Event: PROGRAM_RECEIVED */    FINISHED,
    /* Event: START_PROGRAM_SIGNAL */ FINISHED,
    /* Event: STOP_PROGRAM_SIGNAL */  FINISHED,
    /* Event: PROGRAM_FINISHED */    FINISHED,
    /* Event: PROGRAM_ERROR */       FINISHED,
    /* Event: RESET_PROGRAM */       WAITING_FOR_START
  },
  // Current State: ERROR
  {
    /* Event: CONNECTION_ESTABLISHED */ ERROR,
    /* Event: CONNECTION_LOST */      WAITING_FOR_CONNECTION,
    /* Event: PROGRAM_RECEIVED */    ERROR,
    /* Event: START_PROGRAM_SIGNAL */ ERROR,
    /* Event: STOP_PROGRAM_SIGNAL */  ERROR,
    /* Event: PROGRAM_FINISHED */    ERROR,
    /* Event: PROGRAM_ERROR */       ERROR,
    /* Event: RESET_PROGRAM */       WAITING_FOR_START
  }
};
CommunicationManager commManager;
TemperatureManager tempManager;

QueueHandle_t xQueuDataComm;
QueueHandle_t xQueueStatusComm;
// QueueHandle_t xQueuDataHMI;
// QueueHandle_t  xQueueStatusHMI;
QueueHandle_t  xQueueProgram;

QueueHandle_t xQueuEvent;
SemaphoreHandle_t xStateMutex;
FSMState currentState = WAITING_FOR_CONNECTION;


void taskCommunication(void *pvParameters) {
  static portTickType lastWakeTime;
  static std::pair<float, uint32_t> newOvenData;
  static OvenStatus newOvenStatus;
  lastWakeTime = xTaskGetTickCount();
  FSMEvent event;
  while (true) {
    if(commManager.isConnectionStatusChanged()) {
      event = commManager.isDeviceConnected() ? CONNECTION_ESTABLISHED : CONNECTION_LOST; // Check connection status
      xQueueSend(xQueuEvent, &event, 0); // Send event to
    }
    if(commManager.isprogramReceived()) {
      auto program = commManager.getCurrentCurve(); // Get the current temperature curve
      xQueueSend(xQueueProgram, &program, 0); // Send the program to the queue
      event = PROGRAM_RECEIVED; // Check if a program is received
      xQueueSend(xQueuEvent, &event, 0); // Send event to FSM
    }
    OvenStatus status = commManager.getOvenStatus(); // Get the oven status
    switch(status) {
      case START_STATUS:
        event = START_PROGRAM_SIGNAL; // If the status is START, send the start signal
        xQueueSend(xQueuEvent, &event, 0); // Send event to FSM
        break;
      case STOP_STATUS:
        event = STOP_PROGRAM_SIGNAL; // If the status is STOP, send the stop signal
        xQueueSend(xQueuEvent, &event, 0); // Send event to FSM
        break;
      case RESTART_STATUS:
        event = RESET_PROGRAM; // If the status is RESTART, send the reset signal
        xQueueSend(xQueuEvent, &event, 0); // Send event to FSM
        break;
      default:
        break; // Do nothing for other statuses     
    }
    if(xQueueReceive(xQueuDataComm, &newOvenData, 0)) {
      commManager.pushOvenData(newOvenData.first, newOvenData.second); // Push the oven data to the communication manager
    }
    if(xQueueReceive(xQueueStatusComm, &newOvenStatus, 0)) {
      commManager.pushOvenStatus(newOvenStatus); // Push the oven status to the communication manager
    }
    commManager.sendOvenData(); // Send the oven data to the client
    commManager.sendOvenStatus(); // Send the oven status to the client
    vTaskDelayUntil(&lastWakeTime, 10 / portTICK_PERIOD_MS); // Delay for 10 ms
  }
}

void taskTemperature(void *pvParameters) {
  static portTickType lastWakeTime;
  static std::pair<float, uint32_t> newOvenData;
  lastWakeTime = xTaskGetTickCount();
  FSMEvent event;
  FSMState currentStateCopy;
  TemperatureCurve currentCurve;
  OvenStatus newOvenStatus;
  while (true) {
    xSemaphoreTake(xStateMutex, portMAX_DELAY); // Take the mutex to protect the state change
    currentStateCopy = currentState; // Copy the current state
   
    switch (currentStateCopy) {
      case WAITING_FOR_CONNECTION:
        // Do nothing, waiting for connection
        break;
      case WAITING_FOR_PROGRAM:
        // Do nothing, waiting for program
        break;
      case WAITING_FOR_START:
        if(xQueueReceive(xQueueProgram, &currentCurve, 0)) {
          tempManager.setTemperatureCurve(currentCurve); // Set the temperature curve
        }
        break;
      case RUNNING:
        if(tempManager.getFailStatus()) {
          event = PROGRAM_ERROR; // If there is a failure, send the error event
          xQueueSend(xQueuEvent, &event, 0); // Send event to FSM
        } else if (tempManager.isProgramFinished()) {
          tempManager.stopProgram(); // Stop the temperature program
          event = PROGRAM_FINISHED; // If the program is finished, send the finished event
          xQueueSend(xQueuEvent, &event, 0); // Send event to FSM
        } else {
          tempManager.runProgram(); // Run the temperature program
          newOvenData = tempManager.getHeatingData(); // Get the heating data
          xQueueSend(xQueuDataComm, &newOvenData, 0); // Send the heating data to the communication manager
        }
        break;
      case FINISHED:
        tempManager.stopProgram(); // Stop the temperature program
        newOvenStatus = STOP_STATUS; // Set the oven status to FINISHED
        xQueueSend(xQueueStatusComm, &newOvenStatus, portMAX_DELAY);
        break;
      case ERROR:
        tempManager.stopProgram(); // Stop the temperature program
        newOvenStatus = ERROR_STATUS; // Set the oven status to ERROR
        xQueueSend(xQueueStatusComm, &newOvenStatus, portMAX_DELAY);
      break;
      default:
        Serial.println("Unknown state!"); // Handle unknown state
        break;
    }
    xSemaphoreGive(xStateMutex); // Release the mutex
    vTaskDelayUntil(&lastWakeTime, 10 / portTICK_PERIOD_MS);
  }
}


void taskFSM(void *pvParameters) {
  FSMEvent event;
  while (true) {
    if (xQueueReceive(xQueuEvent, &event, portMAX_DELAY)) {
      xSemaphoreTake(xStateMutex, portMAX_DELAY); // Take the mutex to protect the state change
      FSMState nextState = transitionMatrix[currentState][event];
      Serial.printf("Transitioning from %d to %d on event %d\n", currentState, nextState, event);
      currentState = nextState; // Update the current state
      xSemaphoreGive(xStateMutex); // Release the mutex
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial to initialize
  Serial.println("Starting Oven work!");
  commManager.init();  // Initialize the communication manager
  tempManager.init();  // Initialize the temperature manager
  xQueuDataComm = xQueueCreate(10, sizeof(std::pair<float, uint32_t>)); // Create a queue for oven data
  xQueueStatusComm = xQueueCreate(10, sizeof(OvenStatus)); // Create a queue for oven status
  xQueueProgram = xQueueCreate(10, sizeof(TemperatureCurve)); // Create a queue for oven program
  xStateMutex = xSemaphoreCreateMutex(); // Create a mutex for state protection
  delay(100);
  xTaskCreate(taskCommunication, "Communication", 4096, NULL, 1, NULL );
  xTaskCreate(taskTemperature, "Temperature", 4096, NULL, 1, NULL );
  xTaskCreate(taskFSM, "FSM", 2048, NULL, 2, NULL ); // Add this line!

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

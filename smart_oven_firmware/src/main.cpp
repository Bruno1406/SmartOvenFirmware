/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.2
 * @date 2025-06-24
 * * @copyright Copyright (c) 2025
 * */

#include <Arduino.h>
#include <CommunicationManager.h>
#include <TemperatureManager.h>

#define RED_LED_PIN 0
#define GREEN_LED_PIN 15
#define YELLOW_LED_PIN 2

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
  START_PROGRAM,
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
  },
  // Current State: FINISHED
  {
    /* Event: CONNECTION_ESTABLISHED */ FINISHED,
    /* Event: CONNECTION_LOST */      WAITING_FOR_CONNECTION,
    /* Event: PROGRAM_RECEIVED */    FINISHED,
    /* Event: START_PROGRAM_SIGNAL */ WAITING_FOR_START,
    /* Event: STOP_PROGRAM_SIGNAL */  FINISHED,
    /* Event: PROGRAM_FINISHED */    FINISHED,
    /* Event: PROGRAM_ERROR */       FINISHED,
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
  }
};
CommunicationManager commManager;
TemperatureManager tempManager;

QueueHandle_t xQueuDataComm;
QueueHandle_t xQueueStatusComm;
QueueHandle_t  xQueueProgram;

QueueHandle_t xQueuEvent;
SemaphoreHandle_t xStateMutex;
SemaphoreHandle_t xTempManagerMutex;
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

void taskControlLoop(void *pvParameters) {
  static portTickType lastWakeTime = xTaskGetTickCount();
  const TickType_t controlPeriod = pdMS_TO_TICKS(10);
  FSMEvent event;

  while (true) {
    FSMState currentStateCopy;
    xSemaphoreTake(xStateMutex, portMAX_DELAY);
    currentStateCopy = currentState;
    xSemaphoreGive(xStateMutex);
    
    xSemaphoreTake(xTempManagerMutex, portMAX_DELAY);
    switch (currentStateCopy) {
      case WAITING_FOR_START:
        // Load the new temperature profile when it arrives.
        TemperatureCurve tempCurve;
        if(xQueueReceive(xQueueProgram, &tempCurve, 0)) {
          tempManager.setTemperatureCurve(tempCurve);
        }
        break;

      case RUNNING:
        // This is the core work of this task.
        if (tempManager.getFailStatus()) {
          event = PROGRAM_ERROR;
          xQueueSend(xQueuEvent, &event, 0);
        } else if (tempManager.isProgramFinished()) {
          event = PROGRAM_FINISHED;
          xQueueSend(xQueuEvent, &event, 0);
        } else {
          // Only execute the heater control logic.
          tempManager.runProgram();
        }
        break;

      default:
        // No control actions needed in other states.
        break;
    }
    xSemaphoreGive(xTempManagerMutex);
    vTaskDelayUntil(&lastWakeTime, controlPeriod);
  }
}

void taskDataSampler(void *pvParameters) {
    static portTickType lastWakeTime = xTaskGetTickCount();
    const TickType_t samplingPeriod = pdMS_TO_TICKS(1000);
    FSMState currentStateCopy;

    while (true) {
        xSemaphoreTake(xStateMutex, portMAX_DELAY);
        currentStateCopy = currentState;
        xSemaphoreGive(xStateMutex);

        // Only sample and send data when the program is running.
        if (currentStateCopy == RUNNING) {
            xSemaphoreTake(xTempManagerMutex, portMAX_DELAY);
            std::pair<float, uint32_t> newOvenData = tempManager.getHeatingData();
            xSemaphoreGive(xTempManagerMutex);
            Serial.printf("Current Temperature: %.2f °C, Time: %d ms\n", newOvenData.first, newOvenData.second);
            // Send to communication task using a non-blocking send.
            xQueueSend(xQueuDataComm, &newOvenData, 0);
        }

        vTaskDelayUntil(&lastWakeTime, samplingPeriod);
    }
}

void taskFSM(void *pvParameters) {
  FSMEvent event;
  OvenStatus newOvenStatus;
  const TickType_t queueTimeout = pdMS_TO_TICKS(20);

  while (true) {
    // Block and wait for a new event to process.
    if (xQueueReceive(xQueuEvent, &event, portMAX_DELAY)) {
      xSemaphoreTake(xStateMutex, portMAX_DELAY);

      FSMState previousState = currentState;
      currentState = transitionMatrix[previousState][event];

      // If a state transition actually occurred, execute its entry action.
      if (currentState != previousState) {
        Serial.printf("Transitioning from %d to %d on event %d\n", previousState, currentState, event);
        xSemaphoreTake(xTempManagerMutex, portMAX_DELAY);
        switch (currentState) {
          case WAITING_FOR_START:
            // Entry Action: Ensure the heater is off while waiting for a program/start signal.
            tempManager.stopProgram();
            TemperatureCurve tempCurve;
            if(xQueueReceive(xQueueProgram, &tempCurve, 0)) {
              tempManager.setTemperatureCurve(tempCurve);
            }
            digitalWrite(YELLOW_LED_PIN,HIGH);
            digitalWrite(RED_LED_PIN,HIGH);
            digitalWrite(GREEN_LED_PIN,LOW);
            break;
          
          case FINISHED:
            // Entry Action: Stop the heater and notify the communication task.
            tempManager.stopProgram();
            newOvenStatus = STOP_STATUS;
            xQueueSend(xQueueStatusComm, &newOvenStatus, queueTimeout);
            digitalWrite(YELLOW_LED_PIN,LOW);
            digitalWrite(RED_LED_PIN,LOW);
            digitalWrite(GREEN_LED_PIN,HIGH);
            break;

          case ERROR:
            // Entry Action: Stop the heater and notify the communication task.
            tempManager.stopProgram();
            newOvenStatus = ERROR_STATUS;
            xQueueSend(xQueueStatusComm, &newOvenStatus, queueTimeout);
            digitalWrite(YELLOW_LED_PIN,LOW);
            digitalWrite(RED_LED_PIN,HIGH);
            digitalWrite(GREEN_LED_PIN,LOW);
            break;

          // No specific entry actions needed for other states.
          case WAITING_FOR_CONNECTION:
            digitalWrite(YELLOW_LED_PIN,HIGH);
            digitalWrite(RED_LED_PIN,LOW);
            digitalWrite(GREEN_LED_PIN,LOW);
            break;
          case WAITING_FOR_PROGRAM:
            digitalWrite(YELLOW_LED_PIN,HIGH);
            digitalWrite(RED_LED_PIN,LOW);
            digitalWrite(GREEN_LED_PIN,HIGH);
            break;
          case RUNNING:
            digitalWrite(YELLOW_LED_PIN,HIGH);
            digitalWrite(RED_LED_PIN,HIGH);
            digitalWrite(GREEN_LED_PIN,HIGH);
            break;
          default:
            digitalWrite(YELLOW_LED_PIN,LOW);
            digitalWrite(RED_LED_PIN,LOW);
            digitalWrite(GREEN_LED_PIN,LOW);
            break;
        }
        xSemaphoreGive(xTempManagerMutex);
      }
      
      xSemaphoreGive(xStateMutex);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial to initialize
  Serial.println("Starting Oven work! Version 0.3");
  commManager.init();
  tempManager.init();
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  digitalWrite(YELLOW_LED_PIN,HIGH);
  digitalWrite(RED_LED_PIN,LOW);
  digitalWrite(GREEN_LED_PIN,LOW);
  
  // Create all necessary queues and mutexes.
  xQueuDataComm = xQueueCreate(10, sizeof(std::pair<float, uint32_t>));
  xQueueStatusComm = xQueueCreate(10, sizeof(OvenStatus));
  xQueueProgram = xQueueCreate(1, sizeof(TemperatureCurve)); // A program queue of 1 is sufficient.
  xQueuEvent = xQueueCreate(10, sizeof(FSMEvent));
  xStateMutex = xSemaphoreCreateMutex();
  xTempManagerMutex = xSemaphoreCreateMutex();
  
  // Create all the application tasks.
  xTaskCreate(taskCommunication, "Communication", 4096, NULL, 1, NULL );
  xTaskCreate(taskFSM, "FSM", 2048, NULL, 2, NULL ); 
  xTaskCreate(taskControlLoop, "ControlLoop", 4096, NULL, 2, NULL ); 
  xTaskCreate(taskDataSampler, "DataSampler", 2048, NULL, 1, NULL ); 
}

void loop() {
  // Intentionally empty. The RTOS scheduler handles all work.
}
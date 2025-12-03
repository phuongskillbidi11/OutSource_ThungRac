#ifndef FLOWCONTROLLER_H
#define FLOWCONTROLLER_H

#include <Arduino.h>
#include "FlowConfig.h"
#include "MotorController.h"

enum FlowState {
  FLOW_IDLE,
  FLOW_WAIT_CLEAR
};

struct FlowRuntime {
  FlowState state;
  bool active;
  unsigned long sensorLastDetectedTime;
  bool sensorLastState;
  unsigned long lastPrintTime;
  unsigned long lastDetectPrint;
  
  // Debounce state
  unsigned long lastChangeTime;
  bool lastStableState;
};

// Global flow runtimes (max 10 flows)
extern FlowRuntime flowRuntimes[10];

// Functions
void initFlows();
bool readSensorWithDebounce(int flowIndex);
bool checkLimitSwitch(int flowIndex);
void processFlow(int flowIndex);  
void enableFlow(int flowIndex);
void disableFlow(int flowIndex);
bool isFlowActive(int flowIndex);
void printFlowStatus(int flowIndex);

#endif
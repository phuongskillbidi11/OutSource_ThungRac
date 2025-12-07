#ifndef FLOWCONTROLLER_H
#define FLOWCONTROLLER_H

#include <Arduino.h>
#include "FlowConfig.h"
#include "MotorController.h"


enum FlowState {
  FLOW_IDLE,
  FLOW_WAIT_CLEAR,
  FLOW_LIMIT_RECOVERY,
  FLOW_TOUCH_HOLD,     
  FLOW_RELAY_ACTIVE    
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
  
  // Limit recovery
  unsigned long limitRecoveryStartTime;
  
  //  TOUCH & RELAY STATE ===== 
  unsigned long touchHoldStartTime;   // Thời điểm bắt đầu giữ touch
  unsigned long relayStartTime;       // Thời điểm bật relay
  bool touchHoldTriggered;           // Đã trigger flow chưa
  // ===== END TOUCH & RELAY STATE =====
  bool completionTriggered;  // Đã trigger flow hoàn thành chưa
  unsigned long completionTriggerTime; // Thời điểm trigger flow hoàn thành
  bool completionTriggerPending; // Có trigger hoàn thành đang chờ không
  bool relayTriggerWaiting; 
   unsigned long relayTriggerWaitStart;
};
// Global flow runtimes (max 10 flows)
extern FlowRuntime flowRuntimes[10];
// extern int activeFlowIndex;
// Functions
void initFlows();
bool readSensorWithDebounce(int flowIndex);
int checkLimitSwitch(int flowIndex);  // ← CHANGED: Return int instead of bool
void processFlow(int flowIndex);  
void enableFlow(int flowIndex);
void disableFlow(int flowIndex);
bool isFlowActive(int flowIndex);
void printFlowStatus(int flowIndex);

#endif
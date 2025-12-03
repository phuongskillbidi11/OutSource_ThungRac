#include "FlowController.h"
#include "MotorController.h"

FlowRuntime flowRuntimes[10];

void initFlows() {
  Serial.println("\n╔═══ INIT FLOWS ═══╗");
  
  for (int i = 0; i < flowSysConfig.flowCount; i++) {
    FlowConfigData* fc = &flowSysConfig.flows[i];
    
    // Setup pins
    pinMode(fc->pins.sensor, INPUT);
    pinMode(fc->pins.limit_cw, INPUT_PULLUP);
    pinMode(fc->pins.limit_ccw, INPUT_PULLUP);
    
    // Init runtime state
    flowRuntimes[i].state = FLOW_IDLE;
    flowRuntimes[i].active = fc->enabled;
    flowRuntimes[i].sensorLastDetectedTime = 0;
    flowRuntimes[i].sensorLastState = false;
    flowRuntimes[i].lastPrintTime = 0;
    flowRuntimes[i].lastDetectPrint = 0;
    flowRuntimes[i].lastChangeTime = 0;
    flowRuntimes[i].lastStableState = false;
    
    Serial.printf("║ Flow[%d]: %s\n", i, fc->name.c_str());
    Serial.printf("║   Motor %d | Sensor GPIO%d | %s\n",
      fc->motor_id, fc->pins.sensor, 
      fc->enabled ? "ENABLED" : "DISABLED");
  }
  
  Serial.println("╚═══════════════════╝\n");
}

bool readSensorWithDebounce(int flowIndex) {
  if (flowIndex >= flowSysConfig.flowCount) return false;
  
  FlowConfigData* fc = &flowSysConfig.flows[flowIndex];
  FlowRuntime* fr = &flowRuntimes[flowIndex];
  
  bool currentReading = (digitalRead(fc->pins.sensor) == LOW); // LOW = detected
  
  if (currentReading != fr->lastStableState) {
    if (millis() - fr->lastChangeTime > fc->sensor.debounce_time) {
      fr->lastStableState = currentReading;
      fr->lastChangeTime = millis();
    }
  } else {
    fr->lastChangeTime = millis();
  }
  
  return fr->lastStableState;
}

bool checkLimitSwitch(int flowIndex) {
  if (flowIndex >= flowSysConfig.flowCount) return false;
  
  FlowConfigData* fc = &flowSysConfig.flows[flowIndex];
  
  if (digitalRead(fc->pins.limit_cw) == LOW) {
    Serial.printf("⛔ Flow[%d] LIMIT CW (GPIO%d) TRIGGERED!\n", 
      flowIndex, fc->pins.limit_cw);
    return true;
  }
  
  if (digitalRead(fc->pins.limit_ccw) == LOW) {
    Serial.printf("⛔ Flow[%d] LIMIT CCW (GPIO%d) TRIGGERED!\n", 
      flowIndex, fc->pins.limit_ccw);
    return true;
  }
  
  return false;
}

void processFlow(int flowIndex) {  // <-- Không nhận tham số motor
  if (flowIndex >= flowSysConfig.flowCount) return;
  
  FlowConfigData* fc = &flowSysConfig.flows[flowIndex];
  FlowRuntime* fr = &flowRuntimes[flowIndex];
  
  if (!fr->active) return;
  
  // Lấy motor index từ config
  int motorIndex = fc->motor_id;
  if (motorIndex >= motorCount || !motors[motorIndex].initialized) return;
  
  Motor &motor = motors[motorIndex];  // <-- Tự lấy motor
  
  // Check limit switches
  if (checkLimitSwitch(flowIndex)) {
    motorStop(motorIndex);
    fr->active = false;
    fr->state = FLOW_IDLE;
    Serial.printf("⛔ Flow[%d] ABORTED!\n", flowIndex);
    return;
  }
  
  bool sensorDetected = readSensorWithDebounce(flowIndex);
  
  if (sensorDetected) {
    fr->sensorLastDetectedTime = millis();
  }
  
  switch (fr->state) {
    case FLOW_IDLE:
      if (sensorDetected && !fr->sensorLastState) {
        Serial.printf("\n╔═══ Flow[%d]: OBJECT DETECTED ═══╗\n", flowIndex);
        Serial.printf("║ +%.1f° rotation...\n", fc->movement.angle);
        Serial.println("╚═══════════════════════════════════╝");
        
        float deg = fc->movement.angle;
        int64_t pulses = (int64_t)((deg / 360.0) * motor.pulsesPerRev);
        int64_t targetPos = motor.encoder.getCount() + pulses;
        
        motorMovePID(motorIndex, targetPos);
        
        fr->state = FLOW_WAIT_CLEAR;
        fr->lastPrintTime = millis();
      }
      break;
      
    case FLOW_WAIT_CLEAR:
      if (!sensorDetected) {
        unsigned long timeSinceClear = millis() - fr->sensorLastDetectedTime;
        
        if (timeSinceClear >= fc->sensor.clear_time) {
          Serial.printf("\n╔═══ Flow[%d]: SENSOR CLEAR ═══╗\n", flowIndex);
          Serial.printf("║ -%.1f° rotation...\n", fc->movement.angle);
          Serial.println("╚════════════════════════════════╝");
          
          float deg = -fc->movement.angle;
          int64_t pulses = (int64_t)((deg / 360.0) * motor.pulsesPerRev);
          int64_t targetPos = motor.encoder.getCount() + pulses;
          
          motorMovePID(motorIndex, targetPos);
          
          Serial.printf("✅ Flow[%d]: Cycle complete\n\n", flowIndex);
          fr->state = FLOW_IDLE;
        } else {
          if (millis() - fr->lastPrintTime > 1000) {
            unsigned long remaining = fc->sensor.clear_time - timeSinceClear;
            Serial.printf("⏳ Flow[%d]: Waiting... %lu ms\n", flowIndex, remaining);
            fr->lastPrintTime = millis();
          }
        }
      } else {
        if (millis() - fr->lastDetectPrint > 2000) {
          Serial.printf("👁️ Flow[%d]: Object detected...\n", flowIndex);
          fr->lastDetectPrint = millis();
        }
      }
      break;
  }
  
  fr->sensorLastState = sensorDetected;
}

void enableFlow(int flowIndex) {
  if (flowIndex >= flowSysConfig.flowCount) {
    Serial.printf("❌ Invalid flow index: %d\n", flowIndex);
    return;
  }
  
  flowRuntimes[flowIndex].active = true;
  flowRuntimes[flowIndex].state = FLOW_IDLE;
  Serial.printf("✅ Flow[%d] ENABLED\n", flowIndex);
}

void disableFlow(int flowIndex) {
  if (flowIndex >= flowSysConfig.flowCount) {
    Serial.printf("❌ Invalid flow index: %d\n", flowIndex);
    return;
  }
  
  flowRuntimes[flowIndex].active = false;
  flowRuntimes[flowIndex].state = FLOW_IDLE;
  Serial.printf("✅ Flow[%d] DISABLED\n", flowIndex);
}

bool isFlowActive(int flowIndex) {
  if (flowIndex >= flowSysConfig.flowCount) return false;
  return flowRuntimes[flowIndex].active;
}

void printFlowStatus(int flowIndex) {
  if (flowIndex >= flowSysConfig.flowCount) {
    Serial.printf("❌ Invalid flow index: %d\n", flowIndex);
    return;
  }
  
  FlowConfigData* fc = &flowSysConfig.flows[flowIndex];
  FlowRuntime* fr = &flowRuntimes[flowIndex];
  
  Serial.printf("\n╔═══ Flow[%d] STATUS ═══╗\n", flowIndex);
  Serial.printf("║ Name: %s\n", fc->name.c_str());
  Serial.printf("║ Motor ID: %d\n", fc->motor_id);
  Serial.printf("║ State: %s\n", fr->active ? "ACTIVE" : "INACTIVE");
  Serial.printf("║ Mode: %s\n", 
    fr->state == FLOW_IDLE ? "IDLE" : "WAIT_CLEAR");
  
  bool sensor = digitalRead(fc->pins.sensor) == LOW;
  bool limitCW = digitalRead(fc->pins.limit_cw) == LOW;
  bool limitCCW = digitalRead(fc->pins.limit_ccw) == LOW;
  
  Serial.printf("║ Sensor (GPIO%d): %s\n", 
    fc->pins.sensor, sensor ? "DETECTED" : "CLEAR");
  Serial.printf("║ Limit CW (GPIO%d): %s\n", 
    fc->pins.limit_cw, limitCW ? "TRIGGERED" : "OPEN");
  Serial.printf("║ Limit CCW (GPIO%d): %s\n", 
    fc->pins.limit_ccw, limitCCW ? "TRIGGERED" : "OPEN");
  
  Serial.println("╚═══════════════════╝\n");
}
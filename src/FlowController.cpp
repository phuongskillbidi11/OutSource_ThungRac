#include "FlowController.h"
#include "MotorController.h"
#include "SystemMode.h"

FlowRuntime flowRuntimes[10];
// int activeFlowIndex = -1;
// ===== LIMIT RECOVERY CONFIG =====
#define LIMIT_RECOVERY_ANGLE 5.0  // Quay ngược 5 độ để clear limit
#define LIMIT_RECOVERY_TIMEOUT 2000  // Max 2s để recovery

// ===== HELPER FUNCTION =====
// Clamp target position to motor's soft limits
int64_t clampToSoftLimits(int motorIndex, int64_t targetPos) {
  if (motorIndex >= motorCount || !motors[motorIndex].initialized) {
    return targetPos;
  }
  
  Motor &motor = motors[motorIndex];
  
  // Check soft limits
  if (targetPos > motor.softLimitMax) {
    Serial.printf("⚠️ Target %lld exceeds soft max %lld, clamping\n", 
      targetPos, motor.softLimitMax);
    return motor.softLimitMax;
  }
  
  if (targetPos < motor.softLimitMin) {
    Serial.printf("⚠️ Target %lld below soft min %lld, clamping\n", 
      targetPos, motor.softLimitMin);
    return motor.softLimitMin;
  }
  
  return targetPos; // Within limits
}

void initFlows() {
  Serial.println("\n╔═══ INIT FLOWS ═══╗");
  
  for (int i = 0; i < flowSysConfig.flowCount; i++) {
    FlowConfigData* fc = &flowSysConfig.flows[i];
    
    // ===== FIX: Check valid pins before setup =====
    if(fc->pins.sensor >= 0) {
      pinMode(fc->pins.sensor, INPUT);
    }
    if(fc->pins.limit_cw >= 0) {
      pinMode(fc->pins.limit_cw, INPUT_PULLUP);
    }
    if(fc->pins.limit_ccw >= 0) {
      pinMode(fc->pins.limit_ccw, INPUT_PULLUP);
    }

    if(fc->pins.relay >= 0) {
      pinMode(fc->pins.relay, OUTPUT);
      // Init relay OFF
      digitalWrite(fc->pins.relay, fc->relay.inverted ? HIGH : LOW);
      Serial.printf("║   Relay GPIO%d | %s logic\n", 
        fc->pins.relay, fc->relay.inverted ? "INVERTED" : "NORMAL");
    }
    
    // Init runtime state
    flowRuntimes[i].state = FLOW_IDLE;
    flowRuntimes[i].active = fc->enabled;
    flowRuntimes[i].sensorLastDetectedTime = 0;
    flowRuntimes[i].sensorLastState = false;
    flowRuntimes[i].lastPrintTime = 0;
    flowRuntimes[i].lastDetectPrint = 0;
    flowRuntimes[i].lastChangeTime = 0;
    flowRuntimes[i].lastStableState = false;
    flowRuntimes[i].limitRecoveryStartTime = 0;
    flowRuntimes[i].touchHoldStartTime = 0;
    flowRuntimes[i].relayStartTime = 0;
    flowRuntimes[i].touchHoldTriggered = false;
    if(fc->pins.sensor >= 0) {
      bool currentSensorState = (digitalRead(fc->pins.sensor) == LOW);
      flowRuntimes[i].sensorLastState = currentSensorState;
      flowRuntimes[i].lastStableState = currentSensorState;
      flowRuntimes[i].lastChangeTime = millis();
      
    }
    
    Serial.printf("║ Flow[%d]: %s\n", i, fc->name.c_str());
    Serial.printf("║   Motor %d | Sensor GPIO%d | %s\n",
      fc->motor_id, fc->pins.sensor, 
      fc->enabled ? "ENABLED" : "DISABLED");
    Serial.printf("║   Type: %s\n", fc->sensor.type.c_str());
    
    // Print motor soft limits
    if (fc->motor_id < motorCount && motors[fc->motor_id].initialized) {
      Motor &m = motors[fc->motor_id];
      Serial.printf("║   Soft Limits: [%lld, %lld] pulses\n",
        m.softLimitMin, m.softLimitMax);
    }
  }
  
  Serial.println("╚═══════════════════╝\n");
}

bool readSensorWithDebounce(int flowIndex) {
  if (flowIndex >= flowSysConfig.flowCount) return false;
  
  FlowConfigData* fc = &flowSysConfig.flows[flowIndex];
  FlowRuntime* fr = &flowRuntimes[flowIndex];
  
  // ===== FIX: Check valid sensor pin =====
  if(fc->pins.sensor < 0) return false;
  
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

// Return: 0 = no limit, 1 = CW limit, 2 = CCW limit
int checkLimitSwitch(int flowIndex) {
  if (flowIndex >= flowSysConfig.flowCount) return 0;
  
  FlowConfigData* fc = &flowSysConfig.flows[flowIndex];
  
  // ===== FIX: Check valid limit pins =====
  if (fc->pins.limit_cw >= 0 && digitalRead(fc->pins.limit_cw) == LOW) {
    return 1; // CW limit triggered
  }
  
  if (fc->pins.limit_ccw >= 0 && digitalRead(fc->pins.limit_ccw) == LOW) {
    return 2; // CCW limit triggered
  }
  
  return 0; // No limit
}

void setRelay(int flowIndex, bool state) {
  if (flowIndex >= flowSysConfig.flowCount) return;
  
  FlowConfigData* fc = &flowSysConfig.flows[flowIndex];
  if(fc->pins.relay < 0) return;  // No relay configured
  
  // Apply inverted logic if configured
  bool pinState = fc->relay.inverted ? !state : state;
  digitalWrite(fc->pins.relay, pinState ? HIGH : LOW);
  
  Serial.printf("🔌 Flow[%d] Relay GPIO%d: %s\n", 
    flowIndex, fc->pins.relay, state ? "ON" : "OFF");
}

void processFlow(int flowIndex) {
  // ===== MODE CHECK =====
  if(sysState.currentMode != MODE_AUTO) {
    return;
  }
  
  if (flowIndex >= flowSysConfig.flowCount) return;
  
  FlowConfigData* fc = &flowSysConfig.flows[flowIndex];
  FlowRuntime* fr = &flowRuntimes[flowIndex];
  
  if (!fr->active) return;
  
  // ===== FLOW MUTEX CHECK ===== (THÊM Ở ĐÂY)
  // if(activeFlowIndex != -1 && activeFlowIndex != flowIndex) {
  //   // Another flow is active, skip this flow
  //   return;
  // }
  // ===== END MUTEX CHECK =====
  
  // Get motor instance
  int motorIndex = fc->motor_id;
  if (motorIndex >= motorCount || !motors[motorIndex].initialized) return;
  
  Motor &motor = motors[motorIndex];
  
  // ===== LIMIT SWITCH HANDLING =====
  int limitStatus = checkLimitSwitch(flowIndex);
  
  if (limitStatus != 0 && fr->state != FLOW_LIMIT_RECOVERY) {
    // Limit triggered! Start recovery process
    Serial.printf("\n⚠️ Flow[%d] LIMIT %s (GPIO%d) TRIGGERED!\n", 
      flowIndex,
      limitStatus == 1 ? "CW" : "CCW",
      limitStatus == 1 ? fc->pins.limit_cw : fc->pins.limit_ccw);
    Serial.println("🔄 Starting auto recovery...");
    
    // Stop motor immediately (safety)
    motorStop(motorIndex);
    delay(50);
    
    // Calculate recovery move (reverse direction)
    float recoveryAngle = (limitStatus == 1) ? -LIMIT_RECOVERY_ANGLE : LIMIT_RECOVERY_ANGLE;
    int64_t pulses = (int64_t)((recoveryAngle / 360.0) * motor.pulsesPerRev);
    int64_t targetPos = motor.encoder.getCount() + pulses;
    
    targetPos = clampToSoftLimits(motorIndex, targetPos);
    
    Serial.printf("🔄 Reversing %.1f° to clear limit...\n", abs(recoveryAngle));
    motorMovePID(motorIndex, targetPos);
    
    // Enter recovery state
    fr->state = FLOW_LIMIT_RECOVERY;
    fr->limitRecoveryStartTime = millis();
    return;
  }
  
  // ===== STATE MACHINE =====
  bool sensorDetected = readSensorWithDebounce(flowIndex);
  
  if (sensorDetected) {
    fr->sensorLastDetectedTime = millis();
  }
  
  switch (fr->state) {
    // ═══════════════════════════════════════════════════════
    // FLOW_IDLE - Waiting for sensor trigger
    // ═══════════════════════════════════════════════════════
    case FLOW_IDLE: {
      // ─────────────────────────────────────────────────────
      // TOUCH SENSOR MODE (Flow 1)
      // ─────────────────────────────────────────────────────
      if(fc->sensor.type == "touch") {
        if(sensorDetected && !fr->sensorLastState) {
          // Touch started
          fr->touchHoldStartTime = millis();
          fr->touchHoldTriggered = false;
          Serial.printf("👆 Flow[%d]: Touch started\n", flowIndex);
        }
        else if(sensorDetected && fr->sensorLastState) {
          // Touch holding - check duration
          
          // ===== FIX: Skip if touchHoldStartTime not set =====
          if(fr->touchHoldStartTime == 0) {
            // Sensor was already detected at boot, waiting for release
            break;
          }
          // ===== END FIX =====
          
          unsigned long holdDuration = millis() - fr->touchHoldStartTime;
          
          if(!fr->touchHoldTriggered && holdDuration >= fc->sensor.hold_time) {
            // Hold time reached! Trigger flow
            Serial.printf("\n╔═══ Flow[%d]: TOUCH HOLD TRIGGERED ═══╗\n", flowIndex);
            Serial.printf("║ Held for %lums (required: %lums)\n", 
              holdDuration, fc->sensor.hold_time);
            Serial.printf("║ Moving to +%.1f°...\n", fc->movement.angle);
            Serial.println("╚═══════════════════════════════════════╝");
            
            // ===== SET LOCK ===== (THÊM)
            // activeFlowIndex = flowIndex;
            // ===== END SET LOCK =====
            
            // Calculate target position (from zero to +angle)
            float deg = fc->movement.angle;
            int64_t pulses = (int64_t)((deg / 360.0) * motor.pulsesPerRev);
            int64_t targetPos = 0 + pulses;
            
            targetPos = clampToSoftLimits(motorIndex, targetPos);
            motorMovePID(motorIndex, targetPos);
            
            fr->touchHoldTriggered = true;
            fr->state = FLOW_RELAY_ACTIVE;
            fr->relayStartTime = 0;  // Will be set when reaching target
          }
          else if(!fr->touchHoldTriggered) {
            // Still holding, print progress
            if(millis() - fr->lastPrintTime > 500) {
              unsigned long remaining = fc->sensor.hold_time - holdDuration;
              Serial.printf("⏱️ Flow[%d]: Holding... %lums left\n", flowIndex, remaining);
              fr->lastPrintTime = millis();
            }
          }
        }
        else if(!sensorDetected && fr->sensorLastState) {
          // Touch released before hold time
          if(!fr->touchHoldTriggered) {
            unsigned long holdDuration = millis() - fr->touchHoldStartTime;
            Serial.printf("👆 Flow[%d]: Touch released early (held %lums, need %lums)\n", 
              flowIndex, holdDuration, fc->sensor.hold_time);
          }
        }
      }
      // ─────────────────────────────────────────────────────
      // IR SENSOR MODE (Flow 0) - Original logic
      // ─────────────────────────────────────────────────────
      else {
        if (sensorDetected && !fr->sensorLastState) {
          Serial.printf("\n╔═══ Flow[%d]: OBJECT DETECTED ═══╗\n", flowIndex);
          Serial.printf("║ +%.1f° rotation...\n", fc->movement.angle);
          Serial.println("╚═══════════════════════════════════╝");
          
          // ===== SET LOCK ===== (THÊM)
          // activeFlowIndex = flowIndex;
          // ===== END SET LOCK =====
          
          float deg = fc->movement.angle;
          int64_t pulses = (int64_t)((deg / 360.0) * motor.pulsesPerRev);
          int64_t targetPos = motor.encoder.getCount() + pulses;
          
          targetPos = clampToSoftLimits(motorIndex, targetPos);
          motorMovePID(motorIndex, targetPos);
          
          fr->state = FLOW_WAIT_CLEAR;
          fr->lastPrintTime = millis();
        }
      }
      break;
    }
    
    // ═══════════════════════════════════════════════════════
    // FLOW_WAIT_CLEAR - IR sensor waiting for clear (Flow 0)
    // ═══════════════════════════════════════════════════════
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
        
        targetPos = clampToSoftLimits(motorIndex, targetPos);
        motorMovePID(motorIndex, targetPos);
        
        Serial.printf("✅ Flow[%d]: Cycle complete\n\n", flowIndex);
        
        // ===== RELEASE LOCK ===== (THÊM)
        // activeFlowIndex = -1;
        // ===== END RELEASE LOCK =====
        
        fr->state = FLOW_IDLE;
      } else {
          // Still waiting
          if (millis() - fr->lastPrintTime > 1000) {
            unsigned long remaining = fc->sensor.clear_time - timeSinceClear;
            Serial.printf("⏳ Flow[%d]: Waiting... %lu ms\n", flowIndex, remaining);
            fr->lastPrintTime = millis();
          }
        }
      } else {
        // Object still detected
        if (millis() - fr->lastDetectPrint > 2000) {
          Serial.printf("👁️ Flow[%d]: Object detected...\n", flowIndex);
          fr->lastDetectPrint = millis();
        }
      }
      break;
    
    // ═══════════════════════════════════════════════════════
    // FLOW_LIMIT_RECOVERY - Recovering from limit switch
    // ═══════════════════════════════════════════════════════
    case FLOW_LIMIT_RECOVERY:
      // Check if limit is cleared
      if (limitStatus == 0) {
        Serial.printf("✅ Flow[%d]: Limit cleared!\n", flowIndex);
        
        // Auto return to zero position
        int64_t currentPos = motor.encoder.getCount();
        if (abs(currentPos) > 50) {
          Serial.printf("🔄 Returning to zero from %lld...\n", currentPos);
          motorMovePID(motorIndex, 0);
          
          // Wait for move complete (max 3s)
          unsigned long startWait = millis();
          while (abs(motor.encoder.getCount()) > 20 && millis() - startWait < 3000) {
            delay(10);
          }
          Serial.printf("✅ Zero position: %lld\n", motor.encoder.getCount());
        }
        
        fr->state = FLOW_IDLE;
        fr->limitRecoveryStartTime = 0;
      } 
      // Check timeout
      else if (millis() - fr->limitRecoveryStartTime > LIMIT_RECOVERY_TIMEOUT) {
        Serial.printf("❌ Flow[%d]: Recovery timeout! ABORT!\n", flowIndex);
        motorStop(motorIndex);
        

        // activeFlowIndex = -1;
        
        fr->active = false;
        fr->state = FLOW_IDLE;
      }
      // Still in recovery
      else {
        if (millis() - fr->lastPrintTime > 500) {
          Serial.printf("⏳ Flow[%d]: Clearing limit...\n", flowIndex);
          fr->lastPrintTime = millis();
        }
      }
      break;
    
    // ═══════════════════════════════════════════════════════
    // FLOW_RELAY_ACTIVE - Touch sensor relay control (Flow 1)
    // ═══════════════════════════════════════════════════════
    case FLOW_RELAY_ACTIVE: {
      // RESET if touch released
      if(fc->sensor.type == "touch" && !sensorDetected) {
        Serial.printf("⚠️ Flow[%d]: Touch released before relay, RESET to IDLE\n", flowIndex);
        fr->state = FLOW_IDLE;
        fr->relayStartTime = 0;
        fr->limitRecoveryStartTime = 0;
        fr->touchHoldTriggered = false;
        break;
      }
      
      int64_t currentPos = motor.encoder.getCount();
      int64_t targetPulses = (int64_t)((fc->movement.angle / 360.0) * motor.pulsesPerRev);
      int64_t error = abs(currentPos - targetPulses);
      
      if(fr->relayStartTime == 0 && fr->limitRecoveryStartTime == 0) {
        fr->limitRecoveryStartTime = millis();
      }
      
      if(fr->relayStartTime == 0 && 
        fr->limitRecoveryStartTime > 0 && 
        millis() - fr->limitRecoveryStartTime > 3000) {
        Serial.printf("⚠️ Flow[%d]: Timeout! Accepting position (error: %lld)\n", 
          flowIndex, error);
        setRelay(flowIndex, true);
        fr->relayStartTime = millis();
        fr->limitRecoveryStartTime = 0;
      }
      
      if(error < 50 && fr->relayStartTime == 0) {
        Serial.printf("✅ Flow[%d]: Target reached (pos: %lld)\n", flowIndex, currentPos);
        setRelay(flowIndex, true);
        fr->relayStartTime = millis();
        fr->limitRecoveryStartTime = 0;
      }
      
      if(fr->relayStartTime > 0) {
        unsigned long relayDuration = millis() - fr->relayStartTime;
        
        if(relayDuration >= fc->relay.duration) {
          Serial.printf("⏰ Flow[%d]: Relay timer complete\n", flowIndex);
          setRelay(flowIndex, false);
          
          Serial.printf("🔄 Flow[%d]: Returning to zero...\n", flowIndex);
          motorMovePID(motorIndex, 0);
          
          unsigned long startWait = millis();
          while(abs(motor.encoder.getCount()) > 20 && millis() - startWait < 3000) {
            delay(10);
          }
          
          Serial.printf("✅ Flow[%d]: Cycle complete, position: %lld\n\n", 
            flowIndex, motor.encoder.getCount());
          
          fr->state = FLOW_IDLE;
          fr->relayStartTime = 0;
          fr->limitRecoveryStartTime = 0;
        }
        else {
          if(millis() - fr->lastPrintTime > 500) {
            unsigned long remaining = fc->relay.duration - relayDuration;
            Serial.printf("⏱️ Flow[%d]: Relay active... %lums left\n", 
              flowIndex, remaining);
            fr->lastPrintTime = millis();
          }
        }
      }
      break;  // ← PHẢI CÓ!
    }  // ← ĐÓNG case FLOW_RELAY_ACTIVE
  }
  
  // Update last sensor state
  fr->sensorLastState = sensorDetected;
}
void enableFlow(int flowIndex) {
  if (flowIndex >= flowSysConfig.flowCount) {
    Serial.printf("❌ Invalid flow index: %d\n", flowIndex);
    return;
  }
  
  FlowConfigData* fc = &flowSysConfig.flows[flowIndex];
  FlowRuntime* fr = &flowRuntimes[flowIndex];
  
  // ===== FIX: SYNC SENSOR STATE TRƯỚC KHI ENABLE =====
  // Đọc trạng thái sensor hiện tại để tránh false trigger
  if(fc->pins.sensor >= 0) {
    bool currentState = (digitalRead(fc->pins.sensor) == LOW);
    fr->sensorLastState = currentState;  // ← SYNC STATE!
    fr->lastStableState = currentState;
    fr->lastChangeTime = millis();
    
    Serial.printf("🔄 Flow[%d]: Synced sensor state = %s\n", 
      flowIndex, currentState ? "DETECTED" : "CLEAR");
  }
  // ===== END FIX =====
  
  fr->active = true;
  fr->state = FLOW_IDLE;
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
    fr->state == FLOW_IDLE ? "IDLE" : 
    fr->state == FLOW_WAIT_CLEAR ? "WAIT_CLEAR" : 
    fr->state == FLOW_LIMIT_RECOVERY ? "LIMIT_RECOVERY" :
    fr->state == FLOW_RELAY_ACTIVE ? "RELAY_ACTIVE" : "UNKNOWN");
  
  // ===== FIX: Check valid pins before reading =====
  if(fc->pins.sensor >= 0) {
    bool sensor = digitalRead(fc->pins.sensor) == LOW;
    Serial.printf("║ Sensor (GPIO%d): %s\n", 
      fc->pins.sensor, sensor ? "DETECTED" : "CLEAR");
  }
  
  if(fc->pins.limit_cw >= 0) {
    bool limitCW = digitalRead(fc->pins.limit_cw) == LOW;
    Serial.printf("║ Limit CW (GPIO%d): %s\n", 
      fc->pins.limit_cw, limitCW ? "TRIGGERED" : "OPEN");
  }
  
  if(fc->pins.limit_ccw >= 0) {
    bool limitCCW = digitalRead(fc->pins.limit_ccw) == LOW;
    Serial.printf("║ Limit CCW (GPIO%d): %s\n", 
      fc->pins.limit_ccw, limitCCW ? "TRIGGERED" : "OPEN");
  }
  
  Serial.println("╚═══════════════════╝\n");
}
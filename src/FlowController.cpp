#include "FlowController.h"
#include "MotorController.h"
#include "SystemMode.h"

FlowRuntime flowRuntimes[10];

// ===== LIMIT RECOVERY CONFIG =====
#define LIMIT_RECOVERY_ANGLE 5.0
#define LIMIT_RECOVERY_TIMEOUT 2000

// ===== HELPER FUNCTION =====
int64_t clampToSoftLimits(int motorIndex, int64_t targetPos) {
  if (motorIndex >= motorCount || !motors[motorIndex].initialized) {
    return targetPos;
  }
  
  Motor &motor = motors[motorIndex];
  
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
  
  return targetPos;
}

void initFlows() {
  Serial.println("\n╔═══ INIT FLOWS ═══╗");
  
  for (int i = 0; i < flowSysConfig.flowCount; i++) {
    FlowConfigData* fc = &flowSysConfig.flows[i];
    
    if(fc->pins.sensor >= 0) {
      if(fc->sensor.type == "touch") {
        pinMode(fc->pins.sensor, INPUT_PULLUP);
      } else {
        pinMode(fc->pins.sensor, INPUT);
      }
    }
    if(fc->pins.limit_cw >= 0) {
      pinMode(fc->pins.limit_cw, INPUT_PULLUP);
    }
    if(fc->pins.limit_ccw >= 0) {
      pinMode(fc->pins.limit_ccw, INPUT_PULLUP);
    }
    if(fc->pins.relay_trigger >= 0) {
      pinMode(fc->pins.relay_trigger, INPUT_PULLUP);
      Serial.printf("║   Relay Trigger GPIO%d (read from config)\n", fc->pins.relay_trigger);
    } else {
      Serial.printf("║   Relay Trigger: NOT CONFIGURED (value: %d)\n", fc->pins.relay_trigger);
    }

    if(fc->pins.relay >= 0) {
      pinMode(fc->pins.relay, OUTPUT);
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
    flowRuntimes[i].completionTriggered = false;
    flowRuntimes[i].completionTriggerTime = 0;
    flowRuntimes[i].relayTriggerWaiting = false;
    flowRuntimes[i].relayTriggerWaitStart = 0;
    
    if(fc->pins.sensor >= 0) {
      bool currentSensorState;
      if(fc->sensor.type == "touch") {
        currentSensorState = (digitalRead(fc->pins.sensor) == HIGH);
      } else {
        currentSensorState = (digitalRead(fc->pins.sensor) == LOW);
      }
      
      flowRuntimes[i].sensorLastState = currentSensorState;
      flowRuntimes[i].lastStableState = currentSensorState;
      flowRuntimes[i].lastChangeTime = millis();
      if(fc->sensor.type == "touch") {
        flowRuntimes[i].touchHoldStartTime = 0; 
      }
    }
    
    Serial.printf("║ Flow[%d]: %s\n", i, fc->name.c_str());
    Serial.printf("║   Motor %d | Sensor GPIO%d | %s\n",
      fc->motor_id, fc->pins.sensor, 
      fc->enabled ? "ENABLED" : "DISABLED");
    Serial.printf("║   Type: %s\n", fc->sensor.type.c_str());
    Serial.printf("║   Speed: %d-%d PWM\n", fc->movement.min_speed, fc->movement.max_speed);
    
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
  
  if(fc->pins.sensor < 0) return false;
  
  bool currentReading;
  if(fc->sensor.type == "touch") {
    currentReading = (digitalRead(fc->pins.sensor) == HIGH);
  } else {
    currentReading = (digitalRead(fc->pins.sensor) == LOW);
  }
  
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

int checkLimitSwitch(int flowIndex) {
  if (flowIndex >= flowSysConfig.flowCount) return 0;
  
  FlowConfigData* fc = &flowSysConfig.flows[flowIndex];
  
  if (fc->pins.limit_cw >= 0 && digitalRead(fc->pins.limit_cw) == LOW) {
    return 1;
  }
  
  if (fc->pins.limit_ccw >= 0 && digitalRead(fc->pins.limit_ccw) == LOW) {
    return 2;
  }
  
  return 0;
}

void setRelay(int flowIndex, bool state) {
  if (flowIndex >= flowSysConfig.flowCount) return;
  
  FlowConfigData* fc = &flowSysConfig.flows[flowIndex];
  if(fc->pins.relay < 0) return;
  
  bool pinState = fc->relay.inverted ? !state : state;
  digitalWrite(fc->pins.relay, pinState ? HIGH : LOW);
  
  Serial.printf("🔌 Flow[%d] Relay GPIO%d: %s\n", 
    flowIndex, fc->pins.relay, state ? "ON" : "OFF");
}

void triggerCompletionFlow(int targetFlowIndex, unsigned long delayMs) {
  if (targetFlowIndex >= flowSysConfig.flowCount) return;
  
  FlowRuntime* fr = &flowRuntimes[targetFlowIndex];
  
  fr->completionTriggerPending = true;
  fr->completionTriggerTime = millis() + delayMs;
  
  Serial.printf("⏰ Flow[%d] will trigger in %lums\n", targetFlowIndex, delayMs);
}

void processFlow(int flowIndex) {
  if(sysState.currentMode != MODE_AUTO) {
    return;
  }
  
  if (flowIndex >= flowSysConfig.flowCount) return;
  
  FlowConfigData* fc = &flowSysConfig.flows[flowIndex];
  FlowRuntime* fr = &flowRuntimes[flowIndex];
  
  if (!fr->active) return;
  
  int motorIndex = fc->motor_id;
  if (motorIndex >= motorCount || !motors[motorIndex].initialized) return;
  
  Motor &motor = motors[motorIndex];
  
  // ===== LIMIT SWITCH HANDLING =====
  int limitStatus = checkLimitSwitch(flowIndex);
  
  if (limitStatus != 0 && fr->state != FLOW_LIMIT_RECOVERY) {
    Serial.printf("\n⚠️ Flow[%d] LIMIT %s (GPIO%d) TRIGGERED!\n", 
      flowIndex,
      limitStatus == 1 ? "CW" : "CCW",
      limitStatus == 1 ? fc->pins.limit_cw : fc->pins.limit_ccw);
    Serial.println("🔄 Starting auto recovery...");
    
    motorStop(motorIndex);
    delay(50);
    
    float recoveryAngle = (limitStatus == 1) ? -LIMIT_RECOVERY_ANGLE : LIMIT_RECOVERY_ANGLE;
    int64_t pulses = (int64_t)((recoveryAngle / 360.0) * motor.pulsesPerRev);
    int64_t targetPos = motor.encoder.getCount() + pulses;
    
    targetPos = clampToSoftLimits(motorIndex, targetPos);
    
    Serial.printf("🔄 Reversing %.1f° to clear limit...\n", abs(recoveryAngle));
    motorMovePID(motorIndex, targetPos);
    
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
    case FLOW_IDLE: {
      // ─────────────────────────────────────────────────────────
      // COMPLETION TRIGGER MODE
      // ─────────────────────────────────────────────────────────
      if(fc->sensor.type == "completion") {
        if(fr->completionTriggerPending) {
          if(millis() >= fr->completionTriggerTime) {
            Serial.printf("🔔 Flow[%d]: Completion trigger activated!\n", flowIndex);
            fr->completionTriggerPending = false;
            
            int backup_min = motor.pid.min_output;
            int backup_max = motor.pid.max_output;
            
            motor.pid.min_output = fc->movement.min_speed;
            motor.pid.max_output = fc->movement.max_speed;
            
            Serial.printf("🎯 Flow[%d]: Speed set to %d-%d PWM\n", 
              flowIndex, fc->movement.min_speed, fc->movement.max_speed);
            
            int64_t targetPulses = (int64_t)((fc->movement.angle / 360.0) * motor.pulsesPerRev);
            targetPulses = clampToSoftLimits(motorIndex, targetPulses);
            
            Serial.printf("🎯 Flow[%d]: Moving to %.1f° (%lld pulses)\n", 
              flowIndex, fc->movement.angle, targetPulses);
            
            motorMovePID(motorIndex, targetPulses);
            
            motor.pid.min_output = backup_min;
            motor.pid.max_output = backup_max;
            
            fr->state = FLOW_WAIT_CLEAR;
            fr->sensorLastDetectedTime = millis();
          } else {
            if(millis() - fr->lastPrintTime > 1000) {
              unsigned long remaining = fr->completionTriggerTime - millis();
              Serial.printf("⏰ Flow[%d]: Waiting %lums before trigger...\n", 
                flowIndex, remaining);
              fr->lastPrintTime = millis();
            }
          }
        }
      }
      
      // ─────────────────────────────────────────────────────────
      // TOUCH SENSOR MODE
      // ─────────────────────────────────────────────────────────
      if(fc->sensor.type == "touch") {
        if(sensorDetected && !fr->sensorLastState) {
          fr->touchHoldStartTime = millis();
          fr->touchHoldTriggered = false;
          Serial.printf("👆 Flow[%d]: Touch started\n", flowIndex);
        }
        
        if(sensorDetected) {
          if(fr->touchHoldStartTime > 0) {
            unsigned long holdDuration = millis() - fr->touchHoldStartTime;
            
            if(holdDuration >= fc->sensor.hold_time && !fr->touchHoldTriggered) {
              Serial.printf("✅ Flow[%d]: Hold time reached! Triggering flow...\n", flowIndex);
              
              int backup_min = motor.pid.min_output;
              int backup_max = motor.pid.max_output;
              
              motor.pid.min_output = fc->movement.min_speed;
              motor.pid.max_output = fc->movement.max_speed;
              
              Serial.printf("🎯 Flow[%d]: Speed set to %d-%d PWM\n", 
                flowIndex, fc->movement.min_speed, fc->movement.max_speed);
              
              int64_t targetPulses = (int64_t)((fc->movement.angle / 360.0) * motor.pulsesPerRev);
              targetPulses = clampToSoftLimits(motorIndex, targetPulses);
              
              Serial.printf("🎯 Flow[%d]: Moving to %.1f° (%lld pulses)\n", 
                flowIndex, fc->movement.angle, targetPulses);
              
              motorMovePID(motorIndex, targetPulses);
              
              motor.pid.min_output = backup_min;
              motor.pid.max_output = backup_max;
              
              fr->touchHoldTriggered = true;
              fr->state = FLOW_RELAY_ACTIVE;
            }
            else if(!fr->touchHoldTriggered) {
              if(millis() - fr->lastPrintTime > 200) {
                unsigned long remaining = fc->sensor.hold_time - holdDuration;
                Serial.printf("⏱️ Flow[%d]: Holding... %lums left\n", flowIndex, remaining);
                fr->lastPrintTime = millis();
              }
            }
          }
        }
        else if(fr->sensorLastState && !sensorDetected) {
          Serial.printf("⚠️ Flow[%d]: Touch released before hold time\n", flowIndex);
          fr->touchHoldStartTime = 0;
          fr->touchHoldTriggered = false;
        }
      }
      
      // ─────────────────────────────────────────────────────────
      // IR SENSOR MODE
      // ─────────────────────────────────────────────────────────
      else if(fc->sensor.type == "ir") {
        if (sensorDetected && !fr->sensorLastState) {
          Serial.printf("👁️ Flow[%d]: Object detected!\n", flowIndex);
          
          int backup_min = motor.pid.min_output;
          int backup_max = motor.pid.max_output;
          
          motor.pid.min_output = fc->movement.min_speed;
          motor.pid.max_output = fc->movement.max_speed;
          
          Serial.printf("🎯 Flow[%d]: Speed set to %d-%d PWM\n", 
            flowIndex, fc->movement.min_speed, fc->movement.max_speed);
          
          int64_t targetPulses = (int64_t)((fc->movement.angle / 360.0) * motor.pulsesPerRev);
          targetPulses = clampToSoftLimits(motorIndex, targetPulses);
          
          Serial.printf("🎯 Flow[%d]: Moving to %.1f° (%lld pulses)\n", 
            flowIndex, fc->movement.angle, targetPulses);
          
          motorMovePID(motorIndex, targetPulses);
          
          motor.pid.min_output = backup_min;
          motor.pid.max_output = backup_max;
          
          fr->state = FLOW_WAIT_CLEAR;
          fr->sensorLastDetectedTime = millis();
        }
      }
      break;
    }
    
    case FLOW_WAIT_CLEAR: {
      if (!sensorDetected) {
        unsigned long timeSinceClear = millis() - fr->sensorLastDetectedTime;
        
        if (timeSinceClear >= fc->sensor.clear_time) {
          Serial.printf("✅ Flow[%d]: Clear time complete, returning to zero...\n", flowIndex);
          
          int backup_min = motor.pid.min_output;
          int backup_max = motor.pid.max_output;
          
          motor.pid.min_output = fc->movement.min_speed;
          motor.pid.max_output = fc->movement.max_speed;
          
          motorMovePID(motorIndex, 0);
          
          motor.pid.min_output = backup_min;
          motor.pid.max_output = backup_max;
          
          unsigned long startWait = millis();
          while (abs(motor.encoder.getCount()) > 20 && millis() - startWait < 3000) {
            delay(10);
          }
          
          Serial.printf("✅ Flow[%d]: Zero position: %lld\n", 
            flowIndex, motor.encoder.getCount());
          
          // ===== RELAY TRIGGER CHECK (FOR ALL FLOW TYPES) =====
          if(fc->pins.relay >= 0 && 
             fc->pins.relay_trigger >= 0 && 
             fc->relay.duration > 0) {
            
            Serial.printf("⏳ Flow[%d]: Waiting for relay trigger (GPIO%d)...\n", 
              flowIndex, fc->pins.relay_trigger);
            Serial.printf("   Press button within 10 seconds to activate relay\n");
            
            // Wait up to 10 seconds for button press
            unsigned long waitStart = millis();
            unsigned long timeout = 10000; // 10 seconds
            bool buttonPressed = false;
            
            while(millis() - waitStart < timeout) {
              // Check if button is pressed (LOW = pressed)
              if(digitalRead(fc->pins.relay_trigger) == LOW) {
                buttonPressed = true;
                break;
              }
              
              // Print countdown every second
              if((millis() - waitStart) % 1000 < 10) {
                unsigned long remaining = (timeout - (millis() - waitStart)) / 1000;
                Serial.printf("   ⏱️ %lu seconds left...\n", remaining);
                delay(10); // Small delay to avoid multiple prints
              }
              
              delay(10);
            }
            
            if(buttonPressed) {
              Serial.printf("🔘 Flow[%d]: Button pressed! Activating relay...\n", flowIndex);
              
              // Wait for button release
              Serial.printf("   Release button to start relay timer\n");
              while(digitalRead(fc->pins.relay_trigger) == LOW) {
                delay(10);
              }
              Serial.printf("   Button released!\n");
              
              // Activate relay
              setRelay(flowIndex, true);
              
              // Wait for relay duration
              unsigned long relayStart = millis();
              while(millis() - relayStart < fc->relay.duration) {
                if(millis() - fr->lastPrintTime > 500) {
                  unsigned long remaining = fc->relay.duration - (millis() - relayStart);
                  Serial.printf("⏱️ Flow[%d]: Relay active... %lums left\n", 
                    flowIndex, remaining);
                  fr->lastPrintTime = millis();
                }
                delay(10);
              }
              
              // Turn off relay
              Serial.printf("⏰ Flow[%d]: Relay timer complete\n", flowIndex);
              setRelay(flowIndex, false);
            } else {
              Serial.printf("⏰ Flow[%d]: Timeout! No button press detected\n", flowIndex);
              Serial.printf("ℹ️ Flow[%d]: Skipping relay activation\n", flowIndex);
            }
          }
          // ===== END RELAY TRIGGER =====
          
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
    
    case FLOW_LIMIT_RECOVERY:
      if (limitStatus == 0) {
        Serial.printf("✅ Flow[%d]: Limit cleared!\n", flowIndex);
        
        int64_t currentPos = motor.encoder.getCount();
        if (abs(currentPos) > 50) {
          Serial.printf("🔄 Returning to zero from %lld...\n", currentPos);
          
          int backup_min = motor.pid.min_output;
          int backup_max = motor.pid.max_output;
          
          motor.pid.min_output = fc->movement.min_speed;
          motor.pid.max_output = fc->movement.max_speed;
          
          motorMovePID(motorIndex, 0);
          
          motor.pid.min_output = backup_min;
          motor.pid.max_output = backup_max;
          
          unsigned long startWait = millis();
          while (abs(motor.encoder.getCount()) > 20 && millis() - startWait < 3000) {
            delay(10);
          }
          Serial.printf("✅ Zero position: %lld\n", motor.encoder.getCount());
        }
        
        fr->state = FLOW_IDLE;
        fr->limitRecoveryStartTime = 0;
      } 
      else if (millis() - fr->limitRecoveryStartTime > LIMIT_RECOVERY_TIMEOUT) {
        Serial.printf("❌ Flow[%d]: Recovery timeout! ABORT!\n", flowIndex);
        motorStop(motorIndex);
        
        fr->active = false;
        fr->state = FLOW_IDLE;
      }
      else {
        if (millis() - fr->lastPrintTime > 500) {
          Serial.printf("⏳ Flow[%d]: Clearing limit...\n", flowIndex);
          fr->lastPrintTime = millis();
        }
      }
      break;
    
    case FLOW_RELAY_ACTIVE: {
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
          
          int backup_min = motor.pid.min_output;
          int backup_max = motor.pid.max_output;
          
          motor.pid.min_output = fc->movement.min_speed;
          motor.pid.max_output = fc->movement.max_speed;
          
          motorMovePID(motorIndex, 0);
          
          motor.pid.min_output = backup_min;
          motor.pid.max_output = backup_max;
          
          unsigned long startWait = millis();
          while(abs(motor.encoder.getCount()) > 20 && millis() - startWait < 3000) {
            delay(10);
          }
          
          Serial.printf("✅ Flow[%d]: Cycle complete, position: %lld\n\n", 
            flowIndex, motor.encoder.getCount());
          
          // ===== RELAY TRIGGER CHECK (FOR TOUCH SENSOR FLOWS) =====
          if(fc->pins.relay >= 0 && 
             fc->pins.relay_trigger >= 0 && 
             fc->relay.duration > 0) {
            
            delay(100);
            fr->relayTriggerWaiting = true;
            fr->relayTriggerWaitStart = millis(); 
            Serial.printf("⏳ Flow[%d]: Waiting for relay trigger (GPIO%d)...\n", 
              flowIndex, fc->pins.relay_trigger);
            Serial.printf("   Press button within 10 seconds to activate relay\n");
            
            // Wait up to 10 seconds for button press
            unsigned long waitStart = millis();
            // unsigned long timeout = 10000; // 10 seconds
            unsigned long timeout = fc->relay.trigger_timeout; 
            bool buttonPressed = false;
            
            while(millis() - waitStart < timeout) {
              // Check if button is pressed (LOW = pressed)
              if(digitalRead(fc->pins.relay_trigger) == LOW) {
                buttonPressed = true;
                break;
              }
              
              // Print countdown every second
              if((millis() - waitStart) % 1000 < 10) {
                unsigned long remaining = (timeout - (millis() - waitStart)) / 1000;
                Serial.printf("   ⏱️ %lu seconds left...\n", remaining);
                delay(10);
              }
              
              delay(10);
            }
            fr->relayTriggerWaiting = false;
            fr->relayTriggerWaitStart = 0;
            if(buttonPressed) {
              Serial.printf("🔘 Flow[%d]: Button pressed! Activating relay...\n", flowIndex);
              
              // Wait for button release
              Serial.printf("   Release button to start relay timer\n");
              while(digitalRead(fc->pins.relay_trigger) == LOW) {
                delay(10);
              }
              Serial.printf("   Button released!\n");
              
              // Activate relay
              setRelay(flowIndex, true);
              
              // Wait for relay duration
              unsigned long relayStart = millis();
              while(millis() - relayStart < fc->relay.duration) {
                if(millis() - fr->lastPrintTime > 500) {
                  unsigned long remaining = fc->relay.duration - (millis() - relayStart);
                  Serial.printf("⏱️ Flow[%d]: Relay active... %lums left\n", 
                    flowIndex, remaining);
                  fr->lastPrintTime = millis();
                }
                delay(10);
              }
              
              // Turn off relay
              Serial.printf("⏰ Flow[%d]: Relay timer complete\n", flowIndex);
              setRelay(flowIndex, false);
            } else {
              Serial.printf("⏰ Flow[%d]: Timeout! No button press detected\n", flowIndex);
              Serial.printf("ℹ️ Flow[%d]: Skipping relay activation\n", flowIndex);
            }
          }
          // ===== END RELAY TRIGGER =====
          
          fr->state = FLOW_IDLE;
          fr->relayStartTime = 0;
          fr->limitRecoveryStartTime = 0;
          
          if(flowIndex == 1) {
            triggerCompletionFlow(2, 2000);
          }
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
      break;
    }
  }
  
  fr->sensorLastState = sensorDetected;
}

void enableFlow(int flowIndex) {
  if (flowIndex >= flowSysConfig.flowCount) {
    Serial.printf("❌ Invalid flow index: %d\n", flowIndex);
    return;
  }
  
  FlowConfigData* fc = &flowSysConfig.flows[flowIndex];
  FlowRuntime* fr = &flowRuntimes[flowIndex];
  
  if(fc->pins.sensor >= 0) {
    bool currentState;
    if(fc->sensor.type == "touch") {
      currentState = (digitalRead(fc->pins.sensor) == HIGH);
    } else {
      currentState = (digitalRead(fc->pins.sensor) == LOW);
    }
    
    fr->sensorLastState = currentState;
    fr->lastStableState = currentState;
    fr->lastChangeTime = millis();
    
    if(fc->sensor.type == "touch") {
      fr->touchHoldStartTime = 0;
      fr->touchHoldTriggered = false;
    }
    
    Serial.printf("🔄 Flow[%d]: Synced sensor state = %s\n", 
      flowIndex, currentState ? "DETECTED" : "CLEAR");
  }
  
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
    bool sensor;
    if(fc->sensor.type == "touch") {
      sensor = (digitalRead(fc->pins.sensor) == HIGH);  // Touch: HIGH = chạm
    } else {
      sensor = (digitalRead(fc->pins.sensor) == LOW);   // IR: LOW = chạm
    }
    
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

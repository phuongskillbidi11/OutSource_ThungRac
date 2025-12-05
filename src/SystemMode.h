#ifndef SYSTEMMODE_H
#define SYSTEMMODE_H

#include <Arduino.h>
#include "MotorController.h"
#include "FlowConfig.h"
#include "FlowController.h"

// ===== SYSTEM MODES =====
enum SystemMode {
  MODE_IDLE = 0,      // System ready, no active operation
  MODE_MANUAL = 1,    // Manual control active (Cal Control)
  MODE_AUTO = 2       // Flow automation active
};

// ===== SYSTEM STATE =====
struct SystemState {
  SystemMode currentMode = MODE_IDLE;
  SystemMode requestedMode = MODE_IDLE;
  bool modeTransitionInProgress = false;
  unsigned long lastModeChange = 0;
  unsigned long transitionStartTime = 0;
  int activeFlowCount = 0;
  bool motorsMoving = false;
  String statusMessage = "System Ready";
};

// Global system state
extern SystemState sysState;

// ===== MODE TRANSITION FUNCTIONS =====

// Check if motors are currently moving
inline bool areMotorsMoving() {
  extern int motorCount;
  extern struct Motor motors[];
  
  for(int i = 0; i < motorCount; i++) {
    if(motors[i].currentDir != 0) { // DIR_STOP = 0
      return true;
    }
  }
  return false;
}

// Check if any flow is active
inline int countActiveFlows() {
  extern struct FlowSystemConfig flowSysConfig;
  extern struct FlowRuntime flowRuntimes[];
  
  int count = 0;
  for(int i = 0; i < flowSysConfig.flowCount; i++) {
    if(flowRuntimes[i].active) count++;
  }
  return count;
}

// ===== MODE CHECKS =====

inline bool canSwitchToManual() {
  // Can switch to manual if:
  // 1. No flows are active
  // 2. Motors are not moving
  // 3. Not already in manual mode
  
  if(sysState.currentMode == MODE_MANUAL) {
    sysState.statusMessage = "Already in Manual Mode";
    return false;
  }
  
  sysState.activeFlowCount = countActiveFlows();
  if(sysState.activeFlowCount > 0) {
    sysState.statusMessage = "Cannot switch: Flows are active. Stop flows first.";
    return false;
  }
  
  sysState.motorsMoving = areMotorsMoving();
  if(sysState.motorsMoving) {
    sysState.statusMessage = "Cannot switch: Motors are moving. Stop motors first.";
    return false;
  }
  
  return true;
}

inline bool canSwitchToAuto() {
  // Can switch to auto if:
  // 1. No manual jog active
  // 2. Not already in auto mode
  
  if(sysState.currentMode == MODE_AUTO) {
    sysState.statusMessage = "Already in Auto Mode";
    return false;
  }
  
  sysState.motorsMoving = areMotorsMoving();
  if(sysState.motorsMoving) {
    sysState.statusMessage = "Cannot switch: Motors are moving. Stop motors first.";
    return false;
  }
  
  return true;
}

// ===== MODE TRANSITIONS =====

inline bool switchToManualMode() {
  Serial.println("\n╔═══ SWITCHING TO MANUAL MODE ═══╗");
  
  if(!canSwitchToManual()) {
    Serial.printf("║ ✗ Switch blocked: %s\n", sysState.statusMessage.c_str());
    Serial.println("╚═════════════════════════════════╝\n");
    return false;
  }
  
  // Step 1: Disable all flows
  extern struct FlowSystemConfig flowSysConfig;
  Serial.println("║ Step 1: Disabling all flows...");
  for(int i = 0; i < flowSysConfig.flowCount; i++) {
    extern void disableFlow(int);
    disableFlow(i);
  }
  delay(100);
  
  // Step 2: Verify flows stopped
  sysState.activeFlowCount = countActiveFlows();
  if(sysState.activeFlowCount > 0) {
    Serial.println("║ ✗ Failed to stop all flows");
    Serial.println("╚═════════════════════════════════╝\n");
    return false;
  }
  
  // Step 3: Change mode
  sysState.currentMode = MODE_MANUAL;
  sysState.lastModeChange = millis();
  sysState.statusMessage = "Manual Control Active";
  
  Serial.println("║ ✓ Mode changed to MANUAL");
  Serial.println("║ ⚠ Flow system DISABLED");
  Serial.println("║ ✓ Manual control ENABLED");
  Serial.println("╚═════════════════════════════════╝\n");
  
  return true;
}

inline bool switchToAutoMode() {
  Serial.println("\n╔═══ SWITCHING TO AUTO MODE ═══╗");
  
  if(!canSwitchToAuto()) {
    Serial.printf("║ ✗ Switch blocked: %s\n", sysState.statusMessage.c_str());
    Serial.println("╚═══════════════════════════════╝\n");
    return false;
  }
  
  // Step 1: Stop any jog operations
  extern int motorCount;
  extern void motorStopJog(int);
  Serial.println("║ Step 1: Stopping jog operations...");
  for(int i = 0; i < motorCount; i++) {
    motorStopJog(i);
  }
  delay(100);
  
  // Step 2: Verify motors stopped
  sysState.motorsMoving = areMotorsMoving();
  if(sysState.motorsMoving) {
    Serial.println("║ ✗ Failed to stop all motors");
    Serial.println("╚═════════════════════════════════╝\n");
    return false;
  }
  
  // Step 3: Change mode
  sysState.currentMode = MODE_AUTO;
  sysState.lastModeChange = millis();
  sysState.statusMessage = "Flow Automation Active";
  
  Serial.println("║ ✓ Mode changed to AUTO");
  Serial.println("║ ⚠ Manual control DISABLED");
  Serial.println("║ ✓ Flow system ENABLED");
  Serial.println("╚═════════════════════════════════╝\n");
  
  return true;
}

inline bool switchToIdleMode() {
  Serial.println("\n╔═══ SWITCHING TO IDLE MODE ═══╗");
  
  // Step 1: Stop all operations
  extern int motorCount;
  extern struct FlowSystemConfig flowSysConfig;
  extern void motorStop(int);
  extern void disableFlow(int);
  
  Serial.println("║ Stopping all operations...");
  
  // Stop motors
  for(int i = 0; i < motorCount; i++) {
    motorStop(i);
  }
  
  // Disable flows
  for(int i = 0; i < flowSysConfig.flowCount; i++) {
    disableFlow(i);
  }
  
  delay(200);
  
  // Step 2: Change mode
  sysState.currentMode = MODE_IDLE;
  sysState.lastModeChange = millis();
  sysState.statusMessage = "System Idle";
  sysState.activeFlowCount = 0;
  sysState.motorsMoving = false;
  
  Serial.println("║ ✓ Mode changed to IDLE");
  Serial.println("║ ✓ All operations stopped");
  Serial.println("╚═══════════════════════════════╝\n");
  
  return true;
}

// ===== UTILITY FUNCTIONS =====

inline String getCurrentModeString() {
  switch(sysState.currentMode) {
    case MODE_IDLE:   return "IDLE";
    case MODE_MANUAL: return "MANUAL";
    case MODE_AUTO:   return "AUTO";
    default:          return "UNKNOWN";
  }
}

inline void printSystemState() {
  Serial.println("\n╔═══════ SYSTEM STATE ═══════╗");
  Serial.printf("║ Mode: %s\n", getCurrentModeString().c_str());
  Serial.printf("║ Status: %s\n", sysState.statusMessage.c_str());
  Serial.printf("║ Active Flows: %d\n", countActiveFlows());
  Serial.printf("║ Motors Moving: %s\n", areMotorsMoving() ? "YES" : "NO");
  Serial.println("╚════════════════════════════╝\n");
}

// ===== SAFETY GUARDS =====

// Call this before any manual motor command
inline bool allowManualControl() {
  if(sysState.currentMode == MODE_AUTO) {
    Serial.println("⚠ BLOCKED: Manual control disabled in AUTO mode");
    return false;
  }
  return true;
}

// Call this in processFlow() loop
inline bool allowFlowExecution() {
  if(sysState.currentMode == MODE_MANUAL) {
    return false; // Flows silently disabled in manual mode
  }
  return true;
}

#endif
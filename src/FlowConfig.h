#ifndef FLOWCONFIG_H
#define FLOWCONFIG_H

#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

struct FlowPins {
  int sensor;
  int limit_cw;
  int limit_ccw;
};

struct FlowMovement {
  float angle;
  int timeout;
  int max_speed;
  int min_speed;
};

struct FlowSensor {
  unsigned long clear_time;
  unsigned long debounce_time;
};

struct FlowPID {
  float kp;
  float ki;
  float kd;
};

struct FlowConfigData {
  int id;
  int motor_id;
  bool enabled;
  String name;
  FlowPins pins;
  FlowMovement movement;
  FlowSensor sensor;
  FlowPID pid;
};

struct FlowSystemConfig {
  int flowCount = 0;
  FlowConfigData flows[10];
};

// Global instance
extern FlowSystemConfig flowSysConfig;

inline bool loadFlowConfig() {
  Serial.println("Loading flow config...");
  
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed!");
    return false;
  }
  
  if (!LittleFS.exists("/flow_config.json")) {
    Serial.println("flow_config.json not found, creating default...");
    
    File file = LittleFS.open("/flow_config.json", "w");
    if (!file) {
      Serial.println("Failed to create flow_config.json");
      return false;
    }
    
    file.print(R"({"version":"1.0","flow_count":1,"flows":[{"id":0,"motor_id":0,"enabled":true,"name":"Flow Motor 1","pins":{"sensor":7,"limit_cw":12,"limit_ccw":14},"movement":{"angle":90.0,"timeout":2500,"max_speed":160,"min_speed":195},"sensor":{"clear_time":5000,"debounce_time":100},"pid":{"kp":2.0,"ki":0.01,"kd":10.0}}]})");
    
    file.close();
    Serial.println("Default flow_config.json created");
  }
  
  // Load config
  File file = LittleFS.open("/flow_config.json", "r");
  if (!file) {
    Serial.println("Failed to open flow_config.json");
    return false;
  }
  
  DynamicJsonDocument doc(4096);
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return false;
  }
  
  flowSysConfig.flowCount = doc["flow_count"];
  JsonArray flowsArray = doc["flows"];
  
  Serial.printf("Loading %d flows...\n", flowSysConfig.flowCount);
  
  for (int i = 0; i < flowSysConfig.flowCount && i < 10; i++) {
    JsonObject f = flowsArray[i];
    
    flowSysConfig.flows[i].id = f["id"];
    flowSysConfig.flows[i].motor_id = f["motor_id"];
    flowSysConfig.flows[i].enabled = f["enabled"];
    flowSysConfig.flows[i].name = f["name"].as<String>();
    
    // Pins
    flowSysConfig.flows[i].pins.sensor = f["pins"]["sensor"];
    flowSysConfig.flows[i].pins.limit_cw = f["pins"]["limit_cw"];
    flowSysConfig.flows[i].pins.limit_ccw = f["pins"]["limit_ccw"];
    
    // Movement
    flowSysConfig.flows[i].movement.angle = f["movement"]["angle"];
    flowSysConfig.flows[i].movement.timeout = f["movement"]["timeout"];
    flowSysConfig.flows[i].movement.max_speed = f["movement"]["max_speed"];
    flowSysConfig.flows[i].movement.min_speed = f["movement"]["min_speed"];
    
    // Sensor
    flowSysConfig.flows[i].sensor.clear_time = f["sensor"]["clear_time"];
    flowSysConfig.flows[i].sensor.debounce_time = f["sensor"]["debounce_time"];
    
    // PID
    flowSysConfig.flows[i].pid.kp = f["pid"]["kp"];
    flowSysConfig.flows[i].pid.ki = f["pid"]["ki"];
    flowSysConfig.flows[i].pid.kd = f["pid"]["kd"];
    
    Serial.printf("  [%d] %s - Motor:%d Sensor:%d Speed:%d Angle:%.1f°\n",
      flowSysConfig.flows[i].id, 
      flowSysConfig.flows[i].name.c_str(),
      flowSysConfig.flows[i].motor_id, 
      flowSysConfig.flows[i].pins.sensor,
      flowSysConfig.flows[i].movement.max_speed, 
      flowSysConfig.flows[i].movement.angle);
  }
  
  Serial.println("Flow config loaded!");
  return true;
}

inline void printFlowConfig() {
  Serial.println("\n╔═══ FLOW CONFIG ═══╗");
  Serial.printf("║ Flow Count: %d\n", flowSysConfig.flowCount);
  
  for (int i = 0; i < flowSysConfig.flowCount; i++) {
    Serial.printf("║\n║ [Flow %d] %s\n", 
      flowSysConfig.flows[i].id, 
      flowSysConfig.flows[i].name.c_str());
    Serial.printf("║   Motor ID: %d | %s\n", 
      flowSysConfig.flows[i].motor_id,
      flowSysConfig.flows[i].enabled ? "ENABLED" : "DISABLED");
    Serial.printf("║   Sensor: GPIO%d\n", 
      flowSysConfig.flows[i].pins.sensor);
    Serial.printf("║   Limits: CW=GPIO%d CCW=GPIO%d\n",
      flowSysConfig.flows[i].pins.limit_cw,
      flowSysConfig.flows[i].pins.limit_ccw);
    Serial.printf("║   Angle: %.1f° | Timeout: %dms\n",
      flowSysConfig.flows[i].movement.angle,
      flowSysConfig.flows[i].movement.timeout);
    Serial.printf("║   Speed: %d-%d PWM\n",
      flowSysConfig.flows[i].movement.min_speed,
      flowSysConfig.flows[i].movement.max_speed);
    Serial.printf("║   Clear: %lums | Debounce: %lums\n",
      flowSysConfig.flows[i].sensor.clear_time,
      flowSysConfig.flows[i].sensor.debounce_time);
    Serial.printf("║   PID: Kp=%.2f Ki=%.3f Kd=%.2f\n",
      flowSysConfig.flows[i].pid.kp,
      flowSysConfig.flows[i].pid.ki,
      flowSysConfig.flows[i].pid.kd);
  }
  
  Serial.println("╚═══════════════════╝\n");
}

#endif
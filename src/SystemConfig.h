#ifndef SYSTEMCONFIG_H
#define SYSTEMCONFIG_H

#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "MotorController.h"
struct MotorPins {
  int rl_en;
  int r_pwm;
  int l_pwm;
  int enc_c1;
  int enc_c2;
};

struct MotorConfig {
  int id;
  String name;
  MotorPins pins;
  int pulses_per_rev;
  float kp, ki, kd;
  int64_t soft_min, soft_max;
  int speed_levels[5];  // NEW: PWM values [STOP, SPEED_1, SPEED_2, SPEED_3, SPEED_4]
  int pid_min_output;
  int pid_max_output;
};

struct SystemConfig {
  int motorCount = 0;
  MotorConfig motors[10];  // Max 10 motors
};

// Global config instance
SystemConfig sysConfig;

bool loadSystemConfig() {
  Serial.println("Mounting LittleFS...");
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed!");
    return false;
  }
  
  if (!LittleFS.exists("/config.json")) {
    Serial.println("Config not found, creating default...");
    
    // Create default config
    File file = LittleFS.open("/config.json", "w");
    if (!file) {
      Serial.println("Failed to create config.json");
      return false;
    }
    
    file.print(R"({"version":"1.0","motor_count":3,"motors":[
{"id":0,"name":"Motor 1","pins":{"rl_en":4,"r_pwm":5,"l_pwm":6,"enc_c1":35,"enc_c2":36},"encoder":{"pulses_per_rev":2550},"pid":{"kp":1.0,"ki":0.0,"kd":0.0,"min_output":195,"max_output":255},"limits":{"soft_min":-99999,"soft_max":99999},"speeds":[0,64,128,192,255]},
{"id":1,"name":"Motor 2","pins":{"rl_en":7,"r_pwm":8,"l_pwm":9,"enc_c1":37,"enc_c2":38},"encoder":{"pulses_per_rev":2550},"pid":{"kp":1.0,"ki":0.0,"kd":0.0,"min_output":195,"max_output":255},"limits":{"soft_min":-99999,"soft_max":99999},"speeds":[0,64,128,192,255]},
{"id":2,"name":"Motor 3","pins":{"rl_en":10,"r_pwm":11,"l_pwm":12,"enc_c1":39,"enc_c2":40},"encoder":{"pulses_per_rev":2550},"pid":{"kp":1.0,"ki":0.0,"kd":0.0,"min_output":195,"max_output":255},"limits":{"soft_min":-99999,"soft_max":99999},"speeds":[0,64,128,192,255]}
]})");
    
    file.close();
    Serial.println("Default config.json created");
  }
  
  // Load config
  File file = LittleFS.open("/config.json", "r");
  if (!file) {
    Serial.println("Failed to open config.json");
    return false;
  }
  
//   JsonDocument doc;
DynamicJsonDocument doc(4096);
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return false;
  }
  
  sysConfig.motorCount = doc["motor_count"];
  JsonArray motorsArray = doc["motors"];
  
  Serial.printf("Loading %d motors from config...\n", sysConfig.motorCount);
  
  for (int i = 0; i < sysConfig.motorCount && i < 10; i++) {
    JsonObject m = motorsArray[i];
    sysConfig.motors[i].id = m["id"];
    sysConfig.motors[i].name = m["name"].as<String>();
    
    sysConfig.motors[i].pins.rl_en = m["pins"]["rl_en"];
    sysConfig.motors[i].pins.r_pwm = m["pins"]["r_pwm"];
    sysConfig.motors[i].pins.l_pwm = m["pins"]["l_pwm"];
    sysConfig.motors[i].pins.enc_c1 = m["pins"]["enc_c1"];
    sysConfig.motors[i].pins.enc_c2 = m["pins"]["enc_c2"];
    
    sysConfig.motors[i].pulses_per_rev = m["encoder"]["pulses_per_rev"];
    
    sysConfig.motors[i].kp = m["pid"]["kp"];
    sysConfig.motors[i].ki = m["pid"]["ki"];
    sysConfig.motors[i].kd = m["pid"]["kd"];
    sysConfig.motors[i].pid_min_output = m["pid"]["min_output"] | MIN_STARTUP_SPEED;
    sysConfig.motors[i].pid_max_output = m["pid"]["max_output"] | 255;

    
    sysConfig.motors[i].soft_min = m["limits"]["soft_min"];
    sysConfig.motors[i].soft_max = m["limits"]["soft_max"];
    
    // Load speed levels (with defaults if not present)
    if(m.containsKey("speeds")) {
      JsonArray speedsArray = m["speeds"];
      for(int s = 0; s < 5 && s < speedsArray.size(); s++) {
        sysConfig.motors[i].speed_levels[s] = speedsArray[s];
      }
    } else {
      // Default speeds if not in config
      sysConfig.motors[i].speed_levels[0] = 0;
      sysConfig.motors[i].speed_levels[1] = 64;
      sysConfig.motors[i].speed_levels[2] = 128;
      sysConfig.motors[i].speed_levels[3] = 192;
      sysConfig.motors[i].speed_levels[4] = 255;
    }
    
    Serial.printf("  [%d] %s - EN:%d PWM:%d/%d ENC:%d/%d\n", 
      sysConfig.motors[i].id, sysConfig.motors[i].name.c_str(),
      sysConfig.motors[i].pins.rl_en, sysConfig.motors[i].pins.r_pwm, sysConfig.motors[i].pins.l_pwm,
      sysConfig.motors[i].pins.enc_c1, sysConfig.motors[i].pins.enc_c2);
  }
  
  Serial.println("Config loaded successfully!");
  return true;
}

void printSystemConfig() {
  Serial.println("\n=== SYSTEM CONFIG ===");
  Serial.printf("Motor Count: %d\n", sysConfig.motorCount);
  for (int i = 0; i < sysConfig.motorCount; i++) {
    Serial.printf("\n[Motor %d] %s\n", sysConfig.motors[i].id, sysConfig.motors[i].name.c_str());
    Serial.printf("  Pins: EN=%d, R_PWM=%d, L_PWM=%d\n", 
      sysConfig.motors[i].pins.rl_en, sysConfig.motors[i].pins.r_pwm, sysConfig.motors[i].pins.l_pwm);
    Serial.printf("  Encoder: C1=%d, C2=%d, PPR=%d\n",
      sysConfig.motors[i].pins.enc_c1, sysConfig.motors[i].pins.enc_c2, sysConfig.motors[i].pulses_per_rev);
    Serial.printf("  PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
      sysConfig.motors[i].kp, sysConfig.motors[i].ki, sysConfig.motors[i].kd);
    Serial.printf("  Limits: [%lld, %lld]\n",
      sysConfig.motors[i].soft_min, sysConfig.motors[i].soft_max);
  }
  Serial.println("=====================\n");
}

#endif
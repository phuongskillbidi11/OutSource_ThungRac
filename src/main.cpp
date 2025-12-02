#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "SystemConfig.h"
#include "MotorController.h"
#include "WifiPortal.h"

// ==================== LED RGB ====================
#define RGB_PIN 48
#define NUM_PIXELS 1
Adafruit_NeoPixel pixels(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastBlink = 0;
bool blinkState = false;

// ==================== LED FUNCTIONS ====================
void ledWifiStarting() {
  for (int i = 0; i < 6; i++) {
    pixels.setPixelColor(0, pixels.Color(255, 255, 0));
    pixels.show();
    delay(100);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
    delay(100);
  }
}

void ledWifiReady() {
  for (int i = 0; i < 255; i += 5) {
    pixels.setPixelColor(0, pixels.Color(0, 0, i));
    pixels.show();
    delay(10);
  }
}

void ledUpdate() {
  if (!motorsReady) {
    pixels.setPixelColor(0, pixels.Color(255, 255, 0));
    pixels.show();
    return;
  }
  
  unsigned long currentMillis = millis();
  int currentClients = WiFi.softAPgetStationNum();
  
  if (currentClients == 0) {
    if (currentMillis - lastBlink >= 1000) {
      lastBlink = currentMillis;
      blinkState = !blinkState;
      pixels.setPixelColor(0, blinkState ? pixels.Color(0, 0, 255) : pixels.Color(0, 0, 0));
      pixels.show();
    }
  } else {
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    pixels.show();
  }
}

// ==================== MOTOR RPM TRACKING ====================
void updateMotorRPM() {
  if(!motorsReady) return;
  
  static unsigned long lastUpdate = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastUpdate >= 100) {
    for (int i = 0; i < motorCount; i++) {
      if (i >= 10 || i >= motorCount) continue;
      if (!motors[i].initialized) continue;
      
      Motor &m = motors[i];
      
      if (m.lastSpeedCheck == 0) {
        m.lastSpeedCheck = currentTime;
        continue;
      }
      
      int64_t currentPosition = m.encoder.getCount();
      int64_t pulseCount = currentPosition - m.lastEncoderPosition;
      
      unsigned long dt = currentTime - m.lastSpeedCheck;
      if (dt > 0) {
        float pulsesPerSecond = (pulseCount * 1000.0) / dt;
        m.motorRPM = (pulsesPerSecond * 60.0) / m.pulsesPerRev;
      }
      
      m.lastEncoderPosition = currentPosition;
      m.lastSpeedCheck = currentTime;
    }
    
    lastUpdate = currentTime;
  }
}

// ==================== SERIAL COMMANDS ====================
int selectedMotor = 0;

void printMotorStatus(int index) {
  if(index >= motorCount || !motors[index].initialized) {
    Serial.println("ERROR: Motor not initialized");
    return;
  }
  
  Motor &m = motors[index];
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.printf("║ Motor %d: %s\n", m.id, m.name.c_str());
  Serial.println("╠═══════════════════════════════════════╣");
  Serial.printf("║ Position:     %lld pulses\n", (long long)m.encoder.getCount());
  Serial.printf("║ Angle:        %.2f°\n", motorGetAngle(index));
  Serial.printf("║ RPM:          %.2f\n", m.motorRPM);
  // 🔍 DEBUG: THÊM DÒNG NÀY
  Serial.printf("║ PPR:          %d\n", m.pulsesPerRev);
  Serial.printf("║ Deg/Pulse:    %.6f\n", m.degreesPerPulse);
  Serial.printf("║ Direction:    %s\n", 
    m.currentDir == DIR_CLOCKWISE ? "CW" : 
    m.currentDir == DIR_COUNTER_CLOCKWISE ? "CCW" : "STOP");
  Serial.printf("║ Speed:        %d\n", (int)m.currentSpeed);
  Serial.printf("║ Jog Mode:     %s\n", m.jogMode ? "YES" : "NO");
  Serial.println("╠═══════════════════════════════════════╣");
  Serial.printf("║ PID Moves:    %d\n", m.pid.moveCount);
  Serial.printf("║ Overshoot:    %.1f pulses/rev\n", m.pid.overshootPerRev);
  Serial.printf("║ Warm:         %s\n", m.pid.motorIsWarm ? "YES" : "NO");
  Serial.printf("║ Kp|Ki|Kd:     %.2f | %.3f | %.1f\n", m.pid.Kp, m.pid.Ki, m.pid.Kd);
  Serial.println("╚═══════════════════════════════════════╝\n");
}

void processSerialCommands() {
  if (!Serial.available() || !motorsReady) return;
  
  String input = Serial.readStringUntil('\n');
  input.trim();
  
  if (input.length() == 0) return;
  
  char cmd = input.charAt(0);
  
  if (cmd >= '0' && cmd <= '9') {
    int m = cmd - '0';
    if (m < motorCount && motors[m].initialized) {
      selectedMotor = m;
      Serial.printf("\n✓ Selected Motor %d (%s)\n", 
        motors[selectedMotor].id, motors[selectedMotor].name.c_str());
    } else {
      Serial.printf("✗ Invalid motor %d (valid: 0-%d)\n", m, motorCount - 1);
    }
    return;
  }
  
  input.toUpperCase();
  
  if (input.length() == 1) {
    if (cmd == 'F') {
      motorStartJog(selectedMotor, DIR_CLOCKWISE, SPEED_2);
      Serial.printf("Motor %d: Forward (jog)\n", selectedMotor);
    }
    else if (cmd == 'R') {
      motorStartJog(selectedMotor, DIR_COUNTER_CLOCKWISE, SPEED_2);
      Serial.printf("Motor %d: Reverse (jog)\n", selectedMotor);
    }
    else if (cmd == 'S') {
      motorStop(selectedMotor);
      Serial.printf("Motor %d: Stop\n", selectedMotor);
    }
    else if (cmd == 'Z') {
      motorZeroPosition(selectedMotor);
    }
    else if (cmd == 'P') {
      printMotorStatus(selectedMotor);
    }
    else if (cmd == 'A') {
      for (int i = 0; i < motorCount; i++) {
        motorStop(i);
      }
      Serial.println("✓ All motors stopped");
    }
    else if (cmd == 'I') {
      for (int i = 0; i < motorCount; i++) {
        printMotorStatus(i);
      }
    }
    else if (cmd == 'H') {
      Serial.println("\n╔═══════════════ COMMANDS ═══════════════╗");
      Serial.println("║ Motor Selection:                       ║");
      Serial.println("║   0-9    Select motor                  ║");
      Serial.println("║                                        ║");
      Serial.println("║ Manual Control (selected motor):       ║");
      Serial.println("║   F      Forward (jog)                 ║");
      Serial.println("║   R      Reverse (jog)                 ║");
      Serial.println("║   S      Stop                          ║");
      Serial.println("║                                        ║");
      Serial.println("║ PID Move (selected motor):             ║");
      Serial.println("║   M90    Move by 90 degrees            ║");
      Serial.println("║   A180   Move to absolute 180°         ║");
      Serial.println("║                                        ║");
      Serial.println("║ System:                                ║");
      Serial.println("║   Z      Zero position (selected)      ║");
      Serial.println("║   P      Print status (selected)       ║");
      Serial.println("║   I      Info all motors               ║");
      Serial.println("║   A      Stop all motors               ║");
      Serial.println("║   H      This help                     ║");
      Serial.println("╚════════════════════════════════════════╝\n");
    }
    else {
      Serial.println("✗ Unknown command (H for help)");
    }
    return;
  }
  
  if (input.startsWith("M")) {
    float deg = input.substring(1).toFloat();
    Motor &m = motors[selectedMotor];
    
    // 🔍 DEBUG: THÊM CÁC DÒNG NÀY
    Serial.printf("\n=== M%.1f DEBUG ===\n", deg);
    Serial.printf("Motor PPR: %d\n", m.pulsesPerRev);
    Serial.printf("Current pos: %lld\n", (long long)m.encoder.getCount());
    
    int64_t pulses = (int64_t)((deg / 360.0) * m.pulsesPerRev);
    Serial.printf("Calculated pulses: %lld\n", (long long)pulses);
    
    int64_t targetPos = m.encoder.getCount() + pulses;
    Serial.printf("Target pos: %lld\n", (long long)targetPos);
    Serial.println("==================\n");
    
    motorMovePID(selectedMotor, targetPos);
  }
  else if (input.startsWith("A")) {
    float targetAngle = input.substring(1).toFloat();
    
    Motor &m = motors[selectedMotor];
    
    while(targetAngle < 0) targetAngle += 360;
    while(targetAngle >= 360) targetAngle -= 360;
    
    float currentAngle = motorGetAngle(selectedMotor);
    while(currentAngle < 0) currentAngle += 360;
    while(currentAngle >= 360) currentAngle -= 360;
    
    float angleDiff = targetAngle - currentAngle;
    if(angleDiff > 180) angleDiff -= 360;
    if(angleDiff < -180) angleDiff += 360;
    
    int64_t pulses = (int64_t)((angleDiff / 360.0) * m.pulsesPerRev);
    int64_t targetPos = m.encoder.getCount() + pulses;
    
    Serial.printf("Motor %d: PID move to %.2f° (delta: %.2f°)\n", 
      selectedMotor, targetAngle, angleDiff);
    motorMovePID(selectedMotor, targetPos);
  }
  else if (input.startsWith("TUNE")) {
    int firstSpace = input.indexOf(' ');
    int secondSpace = input.indexOf(' ', firstSpace + 1);
    int thirdSpace = input.indexOf(' ', secondSpace + 1);
    
    if (thirdSpace > 0) {
      float kp = input.substring(firstSpace + 1, secondSpace).toFloat();
      float ki = input.substring(secondSpace + 1, thirdSpace).toFloat();
      float kd = input.substring(thirdSpace + 1).toFloat();
      
      motors[selectedMotor].pid.tune(kp, ki, kd);
    } else {
      Serial.println("✗ Usage: TUNE <Kp> <Ki> <Kd>");
      Serial.println("  Example: TUNE 2.0 0.01 10.0");
    }
  }
  else if (input == "PIDRESET") {
    motors[selectedMotor].pid.reset();
    motors[selectedMotor].pid.resetLearning();
    Serial.printf("Motor %d: PID reset\n", selectedMotor);
  }
  else if (input == "PIDINFO") {
    motors[selectedMotor].pid.printStatus();
  }
  else {
    Serial.println("✗ Unknown command (H for help)");
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   ESP32-S3 Multi-Motor Control System ║");
  Serial.println("║          DEBUG VERSION                 ║");
  Serial.println("╚════════════════════════════════════════╝");
  
  Serial.println("Initializing LED...");
  pixels.begin();
  pixels.setBrightness(50);
  ledWifiStarting();
  
  Serial.println("Loading system config...");
  if (!loadSystemConfig()) {
    Serial.println("\n⌀ ERROR: Config failed!");
    Serial.println("System halted. Check LittleFS and config.json");
    while(1) {
      pixels.setPixelColor(0, pixels.Color(255, 0, 0));
      pixels.show();
      delay(500);
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.show();
      delay(500);
    }
  }
  
  printSystemConfig();
  
  // Initialize motors
  Serial.println("Initializing motors...");
  motorCount = sysConfig.motorCount;

  if (motorCount > 10) {
    Serial.printf("ERROR: motorCount %d > 10!\n", motorCount);
    motorCount = 10;
  }

  Serial.printf("Will initialize %d motors:\n", motorCount);

  for (int i = 0; i < motorCount; i++) {
    Serial.printf("  Setting up motor %d...\n", i);
    
    MotorConfig &mc = sysConfig.motors[i];
    
    // 🔍 DEBUG: THÊM CÁC DÒNG NÀY
    Serial.printf("    Config PPR for M%d: %d\n", i, mc.pulses_per_rev);
    
    motorSetup(
      i,
      mc.id, mc.name,
      mc.pins.rl_en, mc.pins.r_pwm, mc.pins.l_pwm,
      mc.pins.enc_c1, mc.pins.enc_c2,
      mc.pulses_per_rev,
      mc.kp, mc.ki, mc.kd,
      mc.soft_min, mc.soft_max
    );
    
    // 🔍 DEBUG: XÁC NHẬN MOTOR STRUCT ĐÃ CẬP NHẬT
    Serial.printf("    Motor struct PPR for M%d: %d\n", i, motors[i].pulsesPerRev);
    Serial.printf("    Motor struct deg/pulse: %.6f\n", motors[i].degreesPerPulse);
    
    delay(100);
  }

  Serial.printf("✓ All %d motors initialized\n", motorCount);
  
  // Wait for encoders to stabilize
  delay(500);
  
  Serial.println("\nStarting WiFi Portal...");
  WifiPortalsetup();
  delay(100);
  motorsReady = true;
  Serial.println("Motors ready!");
  
  ledWifiReady();
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║            SYSTEM READY                ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.printf("Motors initialized: %d\n", motorCount);
  Serial.printf("WiFi SSID: %s\n", ssid);
  Serial.println("WiFi Password: [Open Network]");
  Serial.println("Web Interface: http://4.3.2.1");
  Serial.println("\n🔍 DEBUG MODE ACTIVE");
  Serial.println("Type 'I' to see all motor PPR values");
  Serial.println("Type 'M90' to test 90° move with debug output");
  Serial.println("\nType 'H' for serial commands help");
  Serial.println("Current motor: 0");
  Serial.println("Ready.\n");
}

// ==================== LOOP ====================
void loop() {
  static bool firstRun = true;
  if (firstRun) {
    delay(1000);
    firstRun = false;
    Serial.println("Loop started!");
  }
  
  if (!motorsReady || motorCount == 0) {
    delay(1000);
    return;
  }
  
  WifiPortalloop();
  yield();
  updateMotorRPM();
  motorsCheckAllSoftLimits();
  
  static unsigned long lastLED = 0;
  if (millis() - lastLED >= 50) {
    lastLED = millis();
    ledUpdate();
  }
  
  processSerialCommands();
  vTaskDelay(1);
}
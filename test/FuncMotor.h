#include <Arduino.h>
#include <ESP32Encoder.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ===== ĐỊNH NGHĨA CHÂN BTS7960 DRIVER =====
#define RL_EN  4
#define R_PWM  5
#define L_PWM  6

// ===== ĐỊNH NGHĨA CHÂN ENCODER =====
#define ENCODER_C1  8
#define ENCODER_C2  9

// ===== CẤU HÌNH PWM =====
#define PWM_FREQ     20000
#define PWM_CHANNEL  0
#define PWM_RESOLUTION 8

// ===== CẤU HÌNH ENCODER =====
#define MOTOR_RPM  30
#define TOTAL_PULSES_PER_REV  2550

// ===== TỐC ĐỘ =====
#define MIN_STARTUP_SPEED 190

enum SpeedLevel {
  SPEED_STOP = 0,
  SPEED_SLOW = 150,
  SPEED_MEDIUM = 190,
  SPEED_FAST = 230,
  SPEED_MAX = 255
};

// ===== HƯỚNG QUAY =====
enum Direction {
  DIR_STOP = 0,
  DIR_CLOCKWISE = 1,
  DIR_COUNTER_CLOCKWISE = 2
};

// ===== HỆ THỐNG BÙ TRỪ =====
struct CompensationSystem {
  float overshootPerRev = 0;
  int crawlSpeedBase = 0;
  int moveCount = 0;
  bool motorIsWarm = false;
  
  float overshootHistory[5] = {0};
  int historyIndex = 0;
  int historyCount = 0;
  
  void addMeasurement(int64_t error, int64_t totalDistance, Direction dir) {
      moveCount++;
      if(moveCount > 2) motorIsWarm = true;
      
      float revolutions = (float)totalDistance / TOTAL_PULSES_PER_REV;
      if(revolutions < 0.1) return;
      
      float errorPerRev = (float)error / revolutions;
      
      Serial.printf("📊 Distance: %.2f revs | Error/rev: %.1f pulses\n", 
        revolutions, errorPerRev);
      
      if(abs(errorPerRev) > 5.0) {
          overshootHistory[historyIndex] = errorPerRev;
          historyIndex = (historyIndex + 1) % 5;
          if(historyCount < 5) historyCount++;
          
          float sum = 0;
          for(int i = 0; i < historyCount; i++) {
              sum += overshootHistory[i];
          }
          overshootPerRev = sum / historyCount;
          
          Serial.printf("⚙️ Learned overshoot/rev: %.1f pulses\n", overshootPerRev);
      }
      
      if(abs(errorPerRev) > 10.0) {
          crawlSpeedBase += 5;
      } else if(abs(errorPerRev) < 5.0 && crawlSpeedBase > 0) {
          crawlSpeedBase -= 1;
      }
      
      crawlSpeedBase = constrain(crawlSpeedBase, -10, 35);
  }
  
  int64_t getPredictedOvershoot(int64_t totalDistance) {
    float revolutions = (float)totalDistance / TOTAL_PULSES_PER_REV;
    return (int64_t)(overshootPerRev * revolutions * 0.85);
  }
  
  int getCrawlSpeed(int64_t totalDistance) {
    int base = MIN_STARTUP_SPEED - 25;
    int warmBoost = motorIsWarm ? 8 : 0;
    float revolutions = (float)totalDistance / TOTAL_PULSES_PER_REV;
    int distanceBoost = (revolutions > 2.0) ? 5 : 0;
    int crawlSpeed = base + crawlSpeedBase + warmBoost + distanceBoost;
    return constrain(crawlSpeed, MIN_STARTUP_SPEED - 30, MIN_STARTUP_SPEED + 20);
  }
  
  void printStatus() {
    Serial.println("\n╔══ COMPENSATION STATUS ═══╗");
    Serial.printf("║ Moves: %d | Warm: %s\n", moveCount, motorIsWarm ? "YES" : "NO");
    Serial.printf("║ Overshoot/rev: %.1f pulses\n", overshootPerRev);
    Serial.printf("║ Crawl speed base: %+d PWM\n", crawlSpeedBase);
    Serial.println("╚═══════════════════════════");
  }
  
  void reset() {
    overshootPerRev = 0;
    crawlSpeedBase = 0;
    moveCount = 0;
    motorIsWarm = false;
    for(int i = 0; i < 5; i++) overshootHistory[i] = 0;
    historyIndex = 0;
    historyCount = 0;
  }
};

CompensationSystem compensation;

// ===== BIẾN TOÀN CỤC =====
ESP32Encoder encoder;
int currentSpeed = 0;
Direction currentDirection = DIR_STOP;
unsigned long lastSpeedCheck = 0;
int64_t lastEncoderPosition = 0;
float motorRPM = 0;

int maxSpeedLimit = 250;          
bool jogMode = false;              
Direction jogDirection = DIR_STOP;  
int jogSpeed = 200;                 
int64_t softLimitMin = -999999999;  
int64_t softLimitMax = 999999999;  
bool softLimitsEnabled = false;

// ✅ ASYNC MOTOR CONTROL
TaskHandle_t motorTaskHandle = NULL;
volatile bool motorTaskRunning = false;
struct MotorCommand {
  int64_t targetPosition;
  int maxSpeed;
  bool active;
} motorCommand = {0, 0, false};

// ===== HÀM TÍNH GÓC/VÒNG =====
float getAngleDegrees() {
  int64_t pos = encoder.getCount();
  int64_t positionMod = pos % TOTAL_PULSES_PER_REV;
  if(positionMod < 0) positionMod += TOTAL_PULSES_PER_REV;
  return (positionMod * 360.0) / (float)TOTAL_PULSES_PER_REV;
}

float getRevolutions() {
  return (float)encoder.getCount() / (float)TOTAL_PULSES_PER_REV;
}

void resetEncoder() {
  encoder.clearCount();
  lastEncoderPosition = 0;
  Serial.println("✓ Encoder reset");
}

void readEncoderSpeed() {
  unsigned long currentTime = millis();
  
  if(currentTime - lastSpeedCheck >= 100) {
    int64_t currentPosition = encoder.getCount();
    int64_t pulseCount = currentPosition - lastEncoderPosition;
    
    float pulsesPerSecond = (pulseCount * 1000.0) / (currentTime - lastSpeedCheck);
    motorRPM = (pulsesPerSecond * 60.0) / TOTAL_PULSES_PER_REV;
    
    lastEncoderPosition = currentPosition;
    lastSpeedCheck = currentTime;
  }
}

void printDetailedStatus() {
  Serial.println("\n╔═══════════════════ STATUS ═══════════════════╗");
  int64_t pos = encoder.getCount();
  Serial.printf("║ Position:     %ld pulses\n", (long)pos);
  Serial.printf("║ Revolutions:  %.3f vòng\n", getRevolutions());
  Serial.printf("║ Angle:        %.2f°\n", getAngleDegrees());
  Serial.printf("║ RPM:          %.2f (target: %d)\n", motorRPM, MOTOR_RPM);
  Serial.printf("║ PWM:          %d (%d%%)\n", currentSpeed, (currentSpeed*100)/255);
  Serial.print("║ Direction:    ");
  Serial.println(currentDirection == DIR_CLOCKWISE ? "CW" : 
                 currentDirection == DIR_COUNTER_CLOCKWISE ? "CCW" : "STOP");
  Serial.println("╚═══════════════════════════════════════════════");
}

void setMotorDirection(Direction dir) {
  currentDirection = dir;
  switch(dir) {
    case DIR_STOP:
      digitalWrite(R_PWM, LOW);
      digitalWrite(L_PWM, LOW);
      break;
    case DIR_CLOCKWISE:
      digitalWrite(R_PWM, LOW);
      digitalWrite(L_PWM, HIGH);
      break;
    case DIR_COUNTER_CLOCKWISE:
      digitalWrite(R_PWM, HIGH);
      digitalWrite(L_PWM, LOW);
      break;
  }
}

void setMotorSpeed(int speed) {
  currentSpeed = constrain(speed, 0, 255);
  ledcWrite(PWM_CHANNEL, currentSpeed);
}

void motorStop() {
  setMotorDirection(DIR_STOP);
  setMotorSpeed(0);
}

void setSpeedLevel(Direction dir, SpeedLevel level) {
  if(level == SPEED_STOP) {
    motorStop();
    return;
  }
  setMotorDirection(dir);
  setMotorSpeed((int)level);
}

// ✅ MOTOR CONTROL TASK - Chạy trên core 0, không block WiFi (core 1)
void motorControlTask(void* parameter) {
  while(true) {
    if(motorCommand.active) {
      motorTaskRunning = true;
      
      int64_t targetPulses = motorCommand.targetPosition;
      int maxSpeed = motorCommand.maxSpeed;
      
      int64_t startPos = encoder.getCount();
      int64_t totalDistance = abs(targetPulses - startPos);
      
      Serial.printf("\n╔═══ MOVE #%d ═══╗\n", compensation.moveCount + 1);
      Serial.printf("║ Target: %ld | Distance: %ld pulses (%.2f revs)\n",
        (long)targetPulses, (long)totalDistance, 
        (float)totalDistance / TOTAL_PULSES_PER_REV);
      
      if(totalDistance >= 5) {
        Direction dir = (targetPulses > startPos) ? DIR_CLOCKWISE : DIR_COUNTER_CLOCKWISE;
        
        int64_t predictedOvershoot = compensation.getPredictedOvershoot(totalDistance);
        int64_t compensatedTarget = targetPulses;
        
        if(abs(predictedOvershoot) > 5 && totalDistance > 500) {
          compensatedTarget = targetPulses - (dir == DIR_CLOCKWISE ? predictedOvershoot : -predictedOvershoot);
          Serial.printf("║ Predicted overshoot: %ld\n", (long)predictedOvershoot);
          Serial.printf("║ Compensated target: %ld\n", (long)compensatedTarget);
        }
        
        Serial.printf("║ Direction: %s | Warm: %s\n",
          dir == DIR_CLOCKWISE ? "CW" : "CCW",
          compensation.motorIsWarm ? "YES" : "NO");
        Serial.println("╚═════════════════════════");
        
        setMotorDirection(dir);
        
        int64_t accelDistance = totalDistance * 2 / 10;
        int64_t decelDistance = totalDistance * 6 / 10;
        
        unsigned long startTime = millis();
        unsigned long lastMovementCheck = millis();
        int64_t lastMovementPos = encoder.getCount();
        int stuckCounter = 0;
        
        // ✅ REDUCED TIMEOUT + STUCK DETECTION
        while(millis() - startTime < 8000 && motorCommand.active) {
          
          // ✅ CRITICAL: Yield frequently to feed watchdog
          vTaskDelay(pdMS_TO_TICKS(10));  // 10ms delay = 100Hz update rate
          
          // Check if motor is stuck
          if(millis() - lastMovementCheck > 500) {
            int64_t currentCheck = encoder.getCount();
            if(abs(currentCheck - lastMovementPos) < 5) {
              stuckCounter++;
              if(stuckCounter >= 4) {  // 2 seconds without movement
                Serial.println("⚠️ Motor stuck - aborting");
                break;
              }
            } else {
              stuckCounter = 0;
            }
            lastMovementPos = currentCheck;
            lastMovementCheck = millis();
          }
          
          readEncoderSpeed();
          int64_t currentPos = encoder.getCount();
          int64_t distanceMoved = abs(currentPos - startPos);
          int64_t distanceToComp = abs(compensatedTarget - currentPos);
          
          int targetSpeed = MIN_STARTUP_SPEED;
          
          if(distanceMoved < accelDistance) {
            float progress = (float)distanceMoved / accelDistance;
            targetSpeed = MIN_STARTUP_SPEED + (int)(progress * (maxSpeed - MIN_STARTUP_SPEED));
          }
          else if(distanceToComp > decelDistance) {
            targetSpeed = maxSpeed;
          }
          else {
            float progress = (float)distanceToComp / decelDistance;
            progress = sqrt(progress);
            targetSpeed = MIN_STARTUP_SPEED + (int)(progress * (maxSpeed - MIN_STARTUP_SPEED));
            
            int adaptiveCrawl = compensation.getCrawlSpeed(totalDistance);
            
            if(distanceToComp < 100) targetSpeed = max(adaptiveCrawl, MIN_STARTUP_SPEED - 30);
            if(distanceToComp < 50) targetSpeed = max(adaptiveCrawl - 8, MIN_STARTUP_SPEED - 30);
            if(distanceToComp < 20) targetSpeed = max(adaptiveCrawl - 15, MIN_STARTUP_SPEED - 30);
            if(distanceToComp < 10) targetSpeed = max(adaptiveCrawl - 20, MIN_STARTUP_SPEED - 30);
            
            targetSpeed = constrain(targetSpeed, MIN_STARTUP_SPEED - 30, MIN_STARTUP_SPEED + 20);
            
            if(distanceToComp <= 2) break;
          }
          
          setMotorSpeed(targetSpeed);
          
          if(dir == DIR_CLOCKWISE && currentPos >= compensatedTarget) break;
          if(dir == DIR_COUNTER_CLOCKWISE && currentPos <= compensatedTarget) break;
        }
        
        motorStop();
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Brake
        Direction brakeDir = (dir == DIR_CLOCKWISE) ? DIR_COUNTER_CLOCKWISE : DIR_CLOCKWISE;
        setMotorDirection(brakeDir);
        setMotorSpeed(MIN_STARTUP_SPEED - 40);
        vTaskDelay(pdMS_TO_TICKS(20));
        motorStop();
        
        vTaskDelay(pdMS_TO_TICKS(500));
        
        int64_t finalPos = encoder.getCount();
        int64_t finalError = finalPos - targetPulses;
        
        compensation.addMeasurement(finalError, totalDistance, dir);
        
        Serial.println("\n╔═══ RESULT ═══╗");
        Serial.printf("║ Target:  %ld\n", (long)targetPulses);
        Serial.printf("║ Actual:  %ld\n", (long)finalPos);
        Serial.printf("║ Error:   %ld pulses (%.2f°)\n",
          (long)finalError, (finalError * 360.0) / TOTAL_PULSES_PER_REV);
        
        if(abs(finalError) <= 5) {
          Serial.println("║ ✅ EXCELLENT!");
        } else if(abs(finalError) <= 15) {
          Serial.println("║ ✅ GOOD!");
        } else {
          Serial.println("║ ⚠️ Learning...");
        }
        Serial.println("╚═══════════════");
      } else {
        Serial.println("║ ✅ Already at target!");
        Serial.println("╚═════════════════════════");
      }
      
      motorCommand.active = false;
      motorTaskRunning = false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));  // Check every 50ms
  }
}

// ===== API FUNCTIONS =====
void moveToPosition(int64_t targetPulses, int maxSpeed) {
  // Wait for previous move to complete
  while(motorTaskRunning) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  if(maxSpeed < MIN_STARTUP_SPEED) {
    maxSpeed = MIN_STARTUP_SPEED;
  }
  
  if(maxSpeed > maxSpeedLimit) {
    maxSpeed = maxSpeedLimit;
  }
  
  if(softLimitsEnabled) {
    if(targetPulses < softLimitMin || targetPulses > softLimitMax) {
      Serial.println("❌ Target exceeds soft limits!");
      return;
    }
  }
  
  // Activate motor command for async task
  motorCommand.targetPosition = targetPulses;
  motorCommand.maxSpeed = maxSpeed;
  motorCommand.active = true;
}

void API_moveByDegrees(float degrees) {
  int64_t pulses = (int64_t)((degrees / 360.0) * TOTAL_PULSES_PER_REV);
  int64_t targetPos = encoder.getCount() + pulses;
  Serial.printf("\n🌐 API: Move by %.2f° (%ld pulses)\n", degrees, (long)pulses);
  moveToPosition(targetPos, 250);
}

void API_moveToAbsoluteDegrees(float degrees) {
  while(degrees < 0) degrees += 360;
  while(degrees >= 360) degrees -= 360;
  
  float currentAngle = getAngleDegrees();
  float angleDiff = degrees - currentAngle;
  
  if(angleDiff > 180) angleDiff -= 360;
  if(angleDiff < -180) angleDiff += 360;
  
  int64_t pulses = (int64_t)((angleDiff / 360.0) * TOTAL_PULSES_PER_REV);
  int64_t targetPos = encoder.getCount() + pulses;
  
  Serial.printf("\n🌐 API: Move to %.2f° (delta %.2f°)\n", degrees, angleDiff);
  moveToPosition(targetPos, 250);
}

void API_moveRevolutions(float revolutions) {
  int64_t pulses = (int64_t)(revolutions * TOTAL_PULSES_PER_REV);
  int64_t targetPos = encoder.getCount() + pulses;
  Serial.printf("\n🌐 API: Move %.2f revs (%ld pulses)\n", revolutions, (long)pulses);
  moveToPosition(targetPos, 250);
}

String API_getCurrentPosition() {
  String json = "{";
  json += "\"position\":" + String((long)encoder.getCount()) + ",";
  json += "\"angle\":" + String(getAngleDegrees(), 2) + ",";
  json += "\"revolutions\":" + String(getRevolutions(), 3) + ",";
  json += "\"rpm\":" + String(motorRPM, 2);
  json += "}";
  return json;
}

void API_resetPosition() {
  Serial.println("\n🌐 API: Reset position");
  resetEncoder();
  compensation.reset();
}

void API_stopMotor() {
  Serial.println("\n🌐 API: Emergency stop");
  motorCommand.active = false;
  motorStop();
}

String API_getStatus() {
  String json = "{";
  json += "\"position\":" + String((long)encoder.getCount()) + ",";
  json += "\"angle\":" + String(getAngleDegrees(), 2) + ",";
  json += "\"revolutions\":" + String(getRevolutions(), 3) + ",";
  json += "\"rpm\":" + String(motorRPM, 2) + ",";
  json += "\"pwm\":" + String(currentSpeed) + ",";
  json += "\"direction\":\"" + String(
    currentDirection == DIR_CLOCKWISE ? "CW" : 
    currentDirection == DIR_COUNTER_CLOCKWISE ? "CCW" : "STOP"
  ) + "\",";
  json += "\"moves\":" + String(compensation.moveCount) + ",";
  json += "\"warm\":" + String(compensation.motorIsWarm ? "true" : "false") + ",";
  json += "\"busy\":" + String(motorTaskRunning ? "true" : "false");
  json += "}";
  return json;
}
// ===== API 9: SET MAX SPEED =====
void API_setMaxSpeed(int speed) {
  maxSpeedLimit = constrain(speed, MIN_STARTUP_SPEED, 255);
  Serial.printf("🌐 API: Max speed set to %d\n", maxSpeedLimit);
}

// ===== API 10: GET COMPENSATION STATUS =====
String API_getCompensationStatus() {
  String json = "{";
  json += "\"overshootPerRev\":" + String(compensation.overshootPerRev, 2) + ",";
  json += "\"crawlSpeedBase\":" + String(compensation.crawlSpeedBase) + ",";
  json += "\"moveCount\":" + String(compensation.moveCount) + ",";
  json += "\"motorWarm\":" + String(compensation.motorIsWarm ? "true" : "false") + ",";
  json += "\"historyCount\":" + String(compensation.historyCount);
  json += "}";
  return json;
}

// ===== API 11: MANUAL SPEED CONTROL =====
void API_setManualSpeed(Direction dir, int speed) {
  if(softLimitsEnabled) {
    int64_t pos = encoder.getCount();
    if((dir == DIR_CLOCKWISE && pos >= softLimitMax) ||
       (dir == DIR_COUNTER_CLOCKWISE && pos <= softLimitMin)) {
      Serial.println("⚠️ Soft limit reached!");
      motorStop();
      return;
    }
  }
  
  setMotorDirection(dir);
  setMotorSpeed(speed);
  Serial.printf("🌐 API: Manual %s @ %d PWM\n", 
    dir == DIR_CLOCKWISE ? "CW" : "CCW", speed);
}

// ===== API 12: JOG MODE =====
void API_startJog(Direction dir, int speed) {
  jogMode = true;
  jogDirection = dir;
  jogSpeed = constrain(speed, MIN_STARTUP_SPEED, 255);
  API_setManualSpeed(dir, jogSpeed);
  Serial.printf("🌐 API: Jog %s started @ %d\n", 
    dir == DIR_CLOCKWISE ? "CW" : "CCW", jogSpeed);
}

void API_stopJog() {
  jogMode = false;
  motorStop();
  Serial.println("🌐 API: Jog stopped");
}

// ===== API 13: EMERGENCY BRAKE =====
void API_emergencyBrake() {
  Serial.println("🌐 API: EMERGENCY BRAKE!");
  
  // Immediate stop
  motorStop();
  
  // Short reverse pulse to brake
  Direction brakeDir = (currentDirection == DIR_CLOCKWISE) ? 
    DIR_COUNTER_CLOCKWISE : DIR_CLOCKWISE;
  
  setMotorDirection(brakeDir);
  setMotorSpeed(255);  // Full power brake
  delay(50);
  motorStop();
  
  // Clear any pending commands (if using async version)
  #ifdef motorCommand
  motorCommand.active = false;
  #endif
}

// ===== API 14: IS MOVING =====
bool API_isMoving() {
  #ifdef motorTaskRunning
  return motorTaskRunning;  // Async version
  #else
  return currentDirection != DIR_STOP;  // Sync version
  #endif
}

// ===== API 15: SET SOFT LIMITS =====
void API_setSoftLimits(int64_t minPos, int64_t maxPos) {
  if(minPos < maxPos) {
    softLimitMin = minPos;
    softLimitMax = maxPos;
    softLimitsEnabled = true;
    Serial.printf("🌐 API: Soft limits set [%ld, %ld]\n", 
      (long)minPos, (long)maxPos);
  } else {
    Serial.println("❌ Invalid limits: min must be < max");
  }
}

void API_disableSoftLimits() {
  softLimitsEnabled = false;
  Serial.println("🌐 API: Soft limits disabled");
}

// ===== API 16: GET ALL CONFIG =====
String API_getAllConfig() {
  String json = "{";
  json += "\"maxSpeed\":" + String(maxSpeedLimit) + ",";
  json += "\"minStartupSpeed\":" + String(MIN_STARTUP_SPEED) + ",";
  json += "\"pulsesPerRev\":" + String(TOTAL_PULSES_PER_REV) + ",";
  json += "\"targetRPM\":" + String(MOTOR_RPM) + ",";
  json += "\"softLimitsEnabled\":" + String(softLimitsEnabled ? "true" : "false") + ",";
  json += "\"softLimitMin\":" + String((long)softLimitMin) + ",";
  json += "\"softLimitMax\":" + String((long)softLimitMax) + ",";
  json += "\"jogMode\":" + String(jogMode ? "true" : "false");
  json += "}";
  return json;
}
// ===== SETUP =====
void motorSetup() {
  if(!Serial) {
    Serial.begin(115200);
    delay(100);
  }
  
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RL_EN, PWM_CHANNEL);
  
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachFullQuad(ENCODER_C1, ENCODER_C2);
  encoder.setFilter(1023);
  encoder.clearCount();
  
  motorStop();
  
  // ✅ CREATE MOTOR TASK ON CORE 0 (WiFi is on core 1)
  xTaskCreatePinnedToCore(
    motorControlTask,
    "MotorTask",
    8192,  // Stack size
    NULL,
    1,     // Priority (lower than WiFi)
    &motorTaskHandle,
    0      // Core 0
  );
  
  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║   ESP32-S3 Motor Control - ASYNC MODE         ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  Serial.println("\n🌐 WEB API Commands:");
  Serial.println("  M90      - Move by 90°");
  Serial.println("  A180     - Move to absolute 180°");
  Serial.println("  R1.5     - Move 1.5 revolutions");
  Serial.println("  POS      - Get position (JSON)");
  Serial.println("  STATUS   - Get full status (JSON)");
  Serial.println("  RESET    - Reset position");
  Serial.println("  STOP     - Emergency stop");
  Serial.printf("\nPPR: %d | Motor: %d RPM\n", TOTAL_PULSES_PER_REV, MOTOR_RPM);
  Serial.println("✅ Async motor task created on Core 0");
  Serial.println("\n>>> Ready! <<<\n");
  
  printDetailedStatus();
}

void processSerialCommands() {
  if(Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    if(cmd.startsWith("M")) {
      float deg = cmd.substring(1).toFloat();
      API_moveByDegrees(deg);
    }
    else if(cmd.startsWith("A")) {
      float deg = cmd.substring(1).toFloat();
      API_moveToAbsoluteDegrees(deg);
    }
    else if(cmd.startsWith("R")) {
      float revs = cmd.substring(1).toFloat();
      API_moveRevolutions(revs);
    }
    else if(cmd == "POS") {
      Serial.println(API_getCurrentPosition());
    }
    else if(cmd == "STATUS") {
      Serial.println(API_getStatus());
    }
    else if(cmd == "RESET") {
      API_resetPosition();
    }
    else if(cmd == "STOP") {
      API_stopMotor();
    }
    else if(cmd == "I") {
      printDetailedStatus();
    }
    else if(cmd == "COMP") {
      compensation.printStatus();
    }
    else if(cmd == "1") {
      setSpeedLevel(DIR_CLOCKWISE, SPEED_SLOW);
      Serial.println("✓ CW SLOW");
    }
    else if(cmd == "2") {
      setSpeedLevel(DIR_CLOCKWISE, SPEED_MEDIUM);
      Serial.println("✓ CW MEDIUM");
    }
    else if(cmd == "3") {
      setSpeedLevel(DIR_CLOCKWISE, SPEED_FAST);
      Serial.println("✓ CW FAST");
    }
    else if(cmd == "4") {
      setSpeedLevel(DIR_COUNTER_CLOCKWISE, SPEED_SLOW);
      Serial.println("✓ CCW SLOW");
    }
    else {
      Serial.println("✗ Unknown command!");
    }
  }
}
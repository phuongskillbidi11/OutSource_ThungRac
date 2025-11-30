#include <Arduino.h>
#include <ESP32Encoder.h>

// ===== ĐỊNH NGHĨA CHÂN BTS7960 DRIVER =====
#define RL_EN  4
#define R_PWM  5
#define L_PWM  6

// ===== ĐỊNH NGHĨA CHÂN ENCODER =====
#define ENCODER_C1  8
#define ENCODER_C2  9

// ===== ĐỊNH NGHĨA CHÂN LIMIT SWITCH =====
#define LIMIT_SWITCH_12 12  // Giới hạn bên trái (CCW)
#define LIMIT_SWITCH_14 14  // Giới hạn bên phải (CW)

// ===== CẤU HÌNH PWM =====
#define PWM_FREQ     20000
#define PWM_CHANNEL  0
#define PWM_RESOLUTION 8

// ===== CẤU HÌNH ENCODER =====
#define MOTOR_RPM  30
#define TOTAL_PULSES_PER_REV  2550

// ===== TỐC ĐỘ =====
#define MIN_STARTUP_SPEED 190

// ===== DEBOUNCE CHO LIMIT SWITCH =====
#define DEBOUNCE_TIME 50

volatile bool limitSwitch12Changed = false;
volatile bool limitSwitch14Changed = false;
volatile unsigned long lastDebounce12 = 0;
volatile unsigned long lastDebounce14 = 0;

bool lastStateLS12 = HIGH;
bool lastStateLS14 = HIGH;

enum SpeedLevel {
  SPEED_STOP = 0,
  SPEED_SLOW = 150,
  SPEED_MEDIUM = 190,
  SPEED_FAST = 230,
  SPEED_MAX = 255
};

enum Direction {
  DIR_STOP = 0,
  DIR_CLOCKWISE = 1,
  DIR_COUNTER_CLOCKWISE = 2
};

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
      
      bool isOvershoot = (dir == DIR_CLOCKWISE && error > 0) || 
                        (dir == DIR_COUNTER_CLOCKWISE && error < 0);
      bool isUndershoot = (dir == DIR_CLOCKWISE && error < 0) || 
                          (dir == DIR_COUNTER_CLOCKWISE && error > 0);
      
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
          
          Serial.printf("⚙️ Learned overshoot/rev: %.1f pulses (avg of %d moves)\n",
            overshootPerRev, historyCount);
      }
      
      if(isUndershoot && abs(errorPerRev) > 10.0) {
          crawlSpeedBase += 8;
          Serial.println("⚙️ Undershoot → crawl +8");
      } 
      else if(isOvershoot && abs(errorPerRev) > 15.0) {
          crawlSpeedBase -= 3;
          Serial.println("⚙️ Overshoot → crawl -3");
      }
      else if(abs(errorPerRev) < 5.0) {
          if(crawlSpeedBase > 5) {
              crawlSpeedBase -= 1;
              Serial.println("✅ Good accuracy, fine-tune crawl -1");
          }
      }
      
      crawlSpeedBase = constrain(crawlSpeedBase, -10, 35);
  }
  
  int64_t getPredictedOvershoot(int64_t totalDistance) {
    float revolutions = (float)totalDistance / TOTAL_PULSES_PER_REV;
    int64_t predicted = (int64_t)(overshootPerRev * revolutions * 0.85);
    return predicted;
  }
  
  int getCrawlSpeed(int64_t totalDistance) {
    int base = MIN_STARTUP_SPEED - 25;
    int warmBoost = motorIsWarm ? 8 : 0;
    
    float revolutions = (float)totalDistance / TOTAL_PULSES_PER_REV;
    int distanceBoost = (revolutions > 2.0) ? 5 : 0;
    
    int crawlSpeed = base + crawlSpeedBase + warmBoost + distanceBoost;
    
    int absoluteMin = MIN_STARTUP_SPEED - 30;
    if(crawlSpeed < absoluteMin) {
        Serial.printf("⚠️ Crawl %d too low! Clamped to %d\n", crawlSpeed, absoluteMin);
        crawlSpeed = absoluteMin;
    }
    
    return crawlSpeed;
  }
  
  void printStatus() {
    Serial.println("\n╔═══ COMPENSATION STATUS ═══╗");
    Serial.printf("║ Moves: %d | Warm: %s\n", moveCount, motorIsWarm ? "YES" : "NO");
    Serial.printf("║ Overshoot/rev: %.1f pulses (learned from %d moves)\n", 
      overshootPerRev, historyCount);
    Serial.printf("║ Crawl speed base: %+d PWM\n", crawlSpeedBase);
    Serial.println("╚═══════════════════════════╝");
  }
  
  void reset() {
    overshootPerRev = 0;
    crawlSpeedBase = 0;
    moveCount = 0;
    motorIsWarm = false;
    for(int i = 0; i < 5; i++) overshootHistory[i] = 0;
    historyIndex = 0;
    historyCount = 0;
    Serial.println("✅ Compensation reset");
  }
};

CompensationSystem compensation;

ESP32Encoder encoder;
int currentSpeed = 0;
Direction currentDirection = DIR_STOP;
unsigned long lastSpeedCheck = 0;
int64_t lastEncoderPosition = 0;
float motorRPM = 0;

// ===== LIMIT SWITCH ISR =====
void IRAM_ATTR handleLimitSwitch12() {
  unsigned long now = millis();
  if (now - lastDebounce12 > DEBOUNCE_TIME) {
    limitSwitch12Changed = true;
    lastDebounce12 = now;
  }
}

void IRAM_ATTR handleLimitSwitch14() {
  unsigned long now = millis();
  if (now - lastDebounce14 > DEBOUNCE_TIME) {
    limitSwitch14Changed = true;
    lastDebounce14 = now;
  }
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
  
  if(speed > 0 && speed < MIN_STARTUP_SPEED - 30) {
    Serial.printf("⚠ Speed %d < SAFE MIN %d\n", speed, MIN_STARTUP_SPEED - 30);
  }
  
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
// ===== KIỂM TRA VÀ IN TRẠNG THÁI LIMIT SWITCH =====
void checkLimitSwitches() {
  if (limitSwitch12Changed) {
    limitSwitch12Changed = false;
    bool currentState = digitalRead(LIMIT_SWITCH_12);
    if (currentState != lastStateLS12) {
      lastStateLS12 = currentState;
      Serial.printf("🔴 Limit Switch 12: %s\n", currentState == HIGH ? "HIGH" : "LOW");
      
      // Dừng motor nếu đang chạy về hướng limit switch
      if (currentState == HIGH && currentDirection == DIR_COUNTER_CLOCKWISE) {
        motorStop();
        Serial.println("⚠️ Motor stopped - Hit limit 12!");
      }
    }
  }
  
  if (limitSwitch14Changed) {
    limitSwitch14Changed = false;
    bool currentState = digitalRead(LIMIT_SWITCH_14);
    if (currentState != lastStateLS14) {
      lastStateLS14 = currentState;
      Serial.printf("🟢 Limit Switch 14: %s\n", currentState == HIGH ? "HIGH" : "LOW");
      
      // Dừng motor nếu đang chạy về hướng limit switch
      if (currentState == HIGH && currentDirection == DIR_CLOCKWISE) {
        motorStop();
        Serial.println("⚠️ Motor stopped - Hit limit 14!");
      }
    }
  }
}

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
  Serial.println("\n╔════════════════════ STATUS ═══════════════════╗");
  
  int64_t pos = encoder.getCount();
  Serial.printf("║ Position:     %ld pulses\n", (long)pos);
  Serial.printf("║ Revolutions:  %.3f vòng\n", getRevolutions());
  Serial.printf("║ Angle:        %.2f°\n", getAngleDegrees());
  Serial.printf("║ RPM:          %.2f (target: %d)\n", motorRPM, MOTOR_RPM);
  Serial.printf("║ PWM:          %d (%d%%)\n", currentSpeed, (currentSpeed*100)/255);
  Serial.print("║ Direction:    ");
  Serial.println(currentDirection == DIR_CLOCKWISE ? "CW" : 
                 currentDirection == DIR_COUNTER_CLOCKWISE ? "CCW" : "STOP");
  Serial.printf("║ Limit 12:     %s\n", digitalRead(LIMIT_SWITCH_12) == HIGH ? "HIGH" : "LOW");
  Serial.printf("║ Limit 14:     %s\n", digitalRead(LIMIT_SWITCH_14) == HIGH ? "HIGH" : "LOW");
  Serial.println("╚═══════════════════════════════════════════════╝");
}

void printMotorStatus() {
  Serial.printf("Pos:%ld | Angle:%.1f° | Rev:%.2f | RPM:%.1f | Speed:%d | Dir:%s | L12:%s | L14:%s\n",
    (long)encoder.getCount(), getAngleDegrees(), getRevolutions(), 
    motorRPM, currentSpeed,
    currentDirection == DIR_CLOCKWISE ? "CW" : 
    currentDirection == DIR_COUNTER_CLOCKWISE ? "CCW" : "STOP",
    digitalRead(LIMIT_SWITCH_12) == HIGH ? "H" : "L",
    digitalRead(LIMIT_SWITCH_14) == HIGH ? "H" : "L");
}

// ===== FUNCTION TEST - TỰ ĐỘNG CHẠY GIỮA 2 LIMIT SWITCH =====
void testLimitSwitches() {
  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║        🔧 LIMIT SWITCH TEST MODE 🔧           ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  
  int64_t backoffDistance = TOTAL_PULSES_PER_REV / 5; // 0.2 vòng
  
  // ===== BƯỚC 1: Chạy CW cho đến khi chạm nút 12 (LOW = pressed) =====
  Serial.println("\n[1] Moving CW until Button 12 pressed (LOW)...");
  setMotorDirection(DIR_CLOCKWISE);
  setMotorSpeed(SPEED_MEDIUM);
  
  unsigned long timeout = millis();
  while(digitalRead(LIMIT_SWITCH_12) == HIGH) {  // Chờ cho đến khi LOW (pressed)
    readEncoderSpeed();
    checkLimitSwitches();
    delay(10);
    
    if(millis() - timeout > 30000) {
      motorStop();
      Serial.println("❌ Timeout - Button 12 not reached!");
      return;
    }
  }
  motorStop();
  Serial.println("✅ Hit Button 12 (pressed = LOW)!");
  delay(500);
  
  // Xịt ra CCW 1 khoảng để nút không còn nhấn (HIGH)
  Serial.println("\n[2] Backing off CCW (release Button 12)...");
  int64_t startPos = encoder.getCount();
  
  setMotorDirection(DIR_COUNTER_CLOCKWISE);
  setMotorSpeed(SPEED_MEDIUM);
  
  // Chạy cho đến khi nút 12 = HIGH (released)
  timeout = millis();
  while(digitalRead(LIMIT_SWITCH_12) == LOW) {  // Chạy cho đến khi HIGH (released)
    readEncoderSpeed();
    delay(10);
    
    if(millis() - timeout > 5000) break;  // Max 5s
  }
  
  // Thêm 1 khoảng nữa để chắc chắn
  delay(200);
  int64_t currentPos = encoder.getCount();
  int64_t targetPos = currentPos - (backoffDistance / 2);
  
  while(encoder.getCount() > targetPos) {
    readEncoderSpeed();
    delay(10);
  }
  motorStop();
  Serial.printf("✅ Backed off %ld pulses (Button 12 released = HIGH)\n", 
    (long)abs(encoder.getCount() - startPos));
  delay(500);
  
  // ===== BƯỚC 2: Chạy CW cho đến khi chạm nút 14 (LOW = pressed) =====
  Serial.println("\n[3] Moving CW until Button 14 pressed (LOW)...");
  setMotorDirection(DIR_CLOCKWISE);
  setMotorSpeed(SPEED_MEDIUM);
  
  timeout = millis();
  while(digitalRead(LIMIT_SWITCH_14) == HIGH) {  // Chờ cho đến khi LOW (pressed)
    readEncoderSpeed();
    checkLimitSwitches();
    delay(10);
    
    if(millis() - timeout > 30000) {
      motorStop();
      Serial.println("❌ Timeout - Button 14 not reached!");
      return;
    }
  }
  motorStop();
  Serial.println("✅ Hit Button 14 (pressed = LOW)!");
  delay(500);
  
  // Xịt ra CCW 1 khoảng để nút không còn nhấn (HIGH)
  Serial.println("\n[4] Backing off CCW (release Button 14)...");
  startPos = encoder.getCount();
  
  setMotorDirection(DIR_COUNTER_CLOCKWISE);
  setMotorSpeed(SPEED_MEDIUM);
  
  // Chạy cho đến khi nút 14 = HIGH (released)
  timeout = millis();
  while(digitalRead(LIMIT_SWITCH_14) == LOW) {  // Chạy cho đến khi HIGH (released)
    readEncoderSpeed();
    delay(10);
    
    if(millis() - timeout > 5000) break;
  }
  
  // Thêm 1 khoảng nữa
  delay(200);
  currentPos = encoder.getCount();
  targetPos = currentPos - (backoffDistance / 2);
  
  while(encoder.getCount() > targetPos) {
    readEncoderSpeed();
    delay(10);
  }
  motorStop();
  int64_t positionAfterButton14 = encoder.getCount();
  Serial.printf("✅ Backed off %ld pulses (Button 14 released = HIGH)\n", 
    (long)abs(encoder.getCount() - startPos));
  
  // ===== BƯỚC 3: Chờ 5 giây =====
  Serial.println("\n[5] Waiting 5 seconds...");
  delay(5000);
  
  // ===== BƯỚC 4: Chạy CCW cho đến khi chạm nút 12 (LOW = pressed) =====
  Serial.println("\n[6] Moving CCW until Button 12 pressed (LOW)...");
  setMotorDirection(DIR_COUNTER_CLOCKWISE);
  setMotorSpeed(SPEED_MEDIUM);
  
  timeout = millis();
  while(digitalRead(LIMIT_SWITCH_12) == HIGH) {  // Chờ cho đến khi LOW (pressed)
    readEncoderSpeed();
    checkLimitSwitches();
    delay(10);
    
    if(millis() - timeout > 30000) {
      motorStop();
      Serial.println("❌ Timeout - Button 12 not reached!");
      return;
    }
  }
  motorStop();
  Serial.println("✅ Hit Button 12 (pressed = LOW)!");
  delay(500);
  
  // Xịt ra CW 1 khoảng để nút không còn nhấn (HIGH)
  Serial.println("\n[7] Backing off CW (release Button 12)...");
  startPos = encoder.getCount();
  
  setMotorDirection(DIR_CLOCKWISE);
  setMotorSpeed(SPEED_MEDIUM);
  
  // Chạy cho đến khi nút 12 = HIGH (released)
  timeout = millis();
  while(digitalRead(LIMIT_SWITCH_12) == LOW) {  // Chạy cho đến khi HIGH (released)
    readEncoderSpeed();
    delay(10);
    
    if(millis() - timeout > 5000) break;
  }
  
  // Thêm 1 khoảng nữa
  delay(200);
  currentPos = encoder.getCount();
  targetPos = currentPos + (backoffDistance / 2);
  
  while(encoder.getCount() < targetPos) {
    readEncoderSpeed();
    delay(10);
  }
  motorStop();
  int64_t positionAfterReturn = encoder.getCount();
  Serial.printf("✅ Backed off %ld pulses (Button 12 released = HIGH)\n", 
    (long)abs(encoder.getCount() - startPos));
  
  // ===== KẾT QUẢ =====
  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║          ✅ TEST COMPLETED! ✅                 ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  Serial.printf("\nTravel distance (Button 12 → 14): %ld pulses (%.2f revs)\n", 
    (long)abs(positionAfterButton14 - positionAfterReturn),
    (float)abs(positionAfterButton14 - positionAfterReturn) / TOTAL_PULSES_PER_REV);
  
  printDetailedStatus();
}
void moveToPosition(int64_t targetPulses, int maxSpeed) {
  if(maxSpeed < MIN_STARTUP_SPEED) {
    maxSpeed = MIN_STARTUP_SPEED;
  }
  
  int64_t startPos = encoder.getCount();
  int64_t totalDistance = abs(targetPulses - startPos);
  
  Serial.printf("\n╔═══ MOVE #%d ═══╗\n", compensation.moveCount + 1);
  Serial.printf("║ Target: %ld | Distance: %ld pulses (%.2f revs)\n",
    (long)targetPulses, (long)totalDistance, 
    (float)totalDistance / TOTAL_PULSES_PER_REV);
  
  if(totalDistance < 5) {
    Serial.println("║ ✅ Already at target!");
    Serial.println("╚═══════════════════════╝");
    return;
  }
  
  Direction dir = (targetPulses > startPos) ? DIR_CLOCKWISE : DIR_COUNTER_CLOCKWISE;
  
  // Kiểm tra limit switch trước khi di chuyển
  if(dir == DIR_CLOCKWISE && digitalRead(LIMIT_SWITCH_12) == HIGH) {
    Serial.println("⚠️ Cannot move CW - at Limit 12!");
    return;
  }
  if(dir == DIR_COUNTER_CLOCKWISE && digitalRead(LIMIT_SWITCH_14) == HIGH) {
    Serial.println("⚠️ Cannot move CCW - at Limit 14!");
    return;
  }
  
  int64_t predictedOvershoot = compensation.getPredictedOvershoot(totalDistance);
  int64_t compensatedTarget = targetPulses;
  
  if(abs(predictedOvershoot) > 5 && totalDistance > 500) {
    compensatedTarget = targetPulses - (dir == DIR_CLOCKWISE ? predictedOvershoot : -predictedOvershoot);
    Serial.printf("║ ⚙️ Predicted overshoot: %ld (%.1f/rev × %.2f revs)\n",
      (long)predictedOvershoot, compensation.overshootPerRev,
      (float)totalDistance / TOTAL_PULSES_PER_REV);
    Serial.printf("║ Compensated target: %ld\n", (long)compensatedTarget);
  }
  
  Serial.printf("║ Direction: %s | Warm: %s\n",
    dir == DIR_CLOCKWISE ? "CW" : "CCW",
    compensation.motorIsWarm ? "YES" : "NO");
  Serial.println("╚═══════════════════════╝");
  
  setMotorDirection(dir);
  
  int64_t accelDistance = totalDistance * 2 / 10;
  int64_t decelDistance = totalDistance * 6 / 10;
  int64_t cruiseDistance = totalDistance - accelDistance - decelDistance;
  
  if(cruiseDistance < 0) {
    accelDistance = totalDistance / 2;
    decelDistance = totalDistance / 2;
    cruiseDistance = 0;
  }
  
  unsigned long startTime = millis();
  int currentSpeed = MIN_STARTUP_SPEED;
  
  while(millis() - startTime < 30000) {
    readEncoderSpeed();
    checkLimitSwitches(); // Kiểm tra limit switch liên tục
    
    // Dừng nếu chạm limit switch
    if((dir == DIR_CLOCKWISE && digitalRead(LIMIT_SWITCH_12) == HIGH) ||
       (dir == DIR_COUNTER_CLOCKWISE && digitalRead(LIMIT_SWITCH_14) == HIGH)) {
      Serial.println("⚠️ Hit limit switch during move!");
      break;
    }
    
    int64_t currentPos = encoder.getCount();
    int64_t distanceMoved = abs(currentPos - startPos);
    int64_t distanceToComp = abs(compensatedTarget - currentPos);
    
    if(distanceMoved < accelDistance) {
      float progress = (float)distanceMoved / accelDistance;
      currentSpeed = MIN_STARTUP_SPEED + (int)(progress * (maxSpeed - MIN_STARTUP_SPEED));
      setMotorSpeed(currentSpeed);
    }
    else if(distanceToComp > decelDistance) {
      setMotorSpeed(maxSpeed);
    }
    else {
      float progress = (float)distanceToComp / decelDistance;
      progress = sqrt(progress);
      currentSpeed = MIN_STARTUP_SPEED + (int)(progress * (maxSpeed - MIN_STARTUP_SPEED));
      
      if(currentSpeed < MIN_STARTUP_SPEED) currentSpeed = MIN_STARTUP_SPEED;
      
      int adaptiveCrawl = compensation.getCrawlSpeed(totalDistance);
      
      if(distanceToComp < 100) {
          currentSpeed = max(adaptiveCrawl, MIN_STARTUP_SPEED - 30);
      }
      if(distanceToComp < 50) {
          currentSpeed = max(adaptiveCrawl - 8, MIN_STARTUP_SPEED - 30);
      }
      if(distanceToComp < 20) {
          currentSpeed = max(adaptiveCrawl - 15, MIN_STARTUP_SPEED - 30);
      }
      if(distanceToComp < 10) {
          currentSpeed = max(adaptiveCrawl - 20, MIN_STARTUP_SPEED - 30);
      }
      
      int minAllowed = MIN_STARTUP_SPEED - 30;
      int maxAllowed = MIN_STARTUP_SPEED + 20;
      currentSpeed = constrain(currentSpeed, minAllowed, maxAllowed);
      
      setMotorSpeed(currentSpeed);
      
      if(distanceToComp <= 2) break;
    }
    
    if(dir == DIR_CLOCKWISE && currentPos >= compensatedTarget) break;
    if(dir == DIR_COUNTER_CLOCKWISE && currentPos <= compensatedTarget) break;
    
    delay(5);
  }
  
  motorStop();
  delay(100);
  
  Direction brakeDir = (dir == DIR_CLOCKWISE) ? DIR_COUNTER_CLOCKWISE : DIR_CLOCKWISE;
  setMotorDirection(brakeDir);
  setMotorSpeed(MIN_STARTUP_SPEED - 40);
  delay(20);
  motorStop();
  
  delay(700);
  
  int64_t finalPos = encoder.getCount();
  int64_t finalError = finalPos - targetPulses;
  
  compensation.addMeasurement(finalError, totalDistance, dir);
  
  Serial.println("\n╔═══ RESULT ═══╗");
  Serial.printf("║ Target:  %ld\n", (long)targetPulses);
  Serial.printf("║ Actual:  %ld\n", (long)finalPos);
  Serial.printf("║ Error:   %ld pulses (%.2f°)\n",
    (long)finalError, (finalError * 360.0) / TOTAL_PULSES_PER_REV);
  Serial.printf("║ Error/rev: %.1f pulses\n", 
    (float)finalError / ((float)totalDistance / TOTAL_PULSES_PER_REV));
  
  if(abs(finalError) <= 5) {
    Serial.println("║ ✅ EXCELLENT!");
  } else if(abs(finalError) <= 15) {
    Serial.println("║ ✅ GOOD!");
  } else {
    Serial.println("║ ⚠️  Learning for next move...");
  }
  Serial.println("╚═══════════════╝");
  
  compensation.printStatus();
  printDetailedStatus();
}

// WEB API FUNCTIONS (giữ nguyên)
void API_moveByDegrees(float degrees) {
  int64_t pulses = (int64_t)((degrees / 360.0) * TOTAL_PULSES_PER_REV);
  int64_t targetPos = encoder.getCount() + pulses;
  
  Serial.printf("\n🌐 API: Move by %.2f degrees (%ld pulses)\n", degrees, (long)pulses);
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
  
  Serial.printf("\n🌐 API: Move to %.2f° (from %.2f°, delta %.2f°)\n", 
    degrees, currentAngle, angleDiff);
  moveToPosition(targetPos, 250);
}

void API_moveRevolutions(float revolutions) {
  int64_t pulses = (int64_t)(revolutions * TOTAL_PULSES_PER_REV);
  int64_t targetPos = encoder.getCount() + pulses;
  
  Serial.printf("\n🌐 API: Move %.2f revolutions (%ld pulses)\n", 
    revolutions, (long)pulses);
  moveToPosition(targetPos, 250);
}

String API_getCurrentPosition() {
  String json = "{";
  json += "\"position\":" + String((long)encoder.getCount()) + ",";
  json += "\"angle\":" + String(getAngleDegrees(), 2) + ",";
  json += "\"revolutions\":" + String(getRevolutions(), 3) + ",";
  json += "\"rpm\":" + String(motorRPM, 2) + ",";
  json += "\"limit12\":" + String(digitalRead(LIMIT_SWITCH_12) == HIGH ? "true" : "false") + ",";
  json += "\"limit14\":" + String(digitalRead(LIMIT_SWITCH_14) == HIGH ? "true" : "false");
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
  motorStop();
}

void API_moveToAbsolutePosition(int64_t position) {
  Serial.printf("\n🌐 API: Move to absolute position %ld\n", (long)position);
  moveToPosition(position, 250);
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
  json += "\"limit12\":" + String(digitalRead(LIMIT_SWITCH_12) == HIGH ? "true" : "false") + ",";
  json += "\"limit14\":" + String(digitalRead(LIMIT_SWITCH_14) == HIGH ? "true" : "false");
  json += "}";
  return json;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  
  // Cấu hình limit switches
  pinMode(LIMIT_SWITCH_12, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_14, INPUT_PULLUP);
  
  // Gắn ngắt cho limit switches
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_12), handleLimitSwitch12, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_14), handleLimitSwitch14, CHANGE);
  
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RL_EN, PWM_CHANNEL);
  
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachFullQuad(ENCODER_C1, ENCODER_C2);
  encoder.setFilter(1023);
  encoder.clearCount();
  
  motorStop();
  
  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║   ESP32-S3 Motor Control - LIMIT SWITCHES     ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  Serial.println("\n🌐 Commands:");
  Serial.println("  TEST     - Run limit switch test");
  Serial.println("  M90      - Move by 90°");
  Serial.println("  M180     - Move by 180°");
  Serial.println("  A90      - Move to absolute 90°");
  Serial.println("  A0       - Move to absolute 0°");
  Serial.println("  R1.5     - Move 1.5 revolutions");
  Serial.println("  POS      - Get current position (JSON)");
  Serial.println("  STATUS   - Get full status (JSON)");
  Serial.println("  RESET    - Reset position");
  Serial.println("  STOP     - Emergency stop");
  Serial.printf("\nPPR: %d | Motor: %d RPM\n", TOTAL_PULSES_PER_REV, MOTOR_RPM);
  Serial.println("\n>>> Ready! <<<\n");
  
  printDetailedStatus();
}

void loop() {
  readEncoderSpeed();
  checkLimitSwitches(); // Kiểm tra limit switches liên tục
  
  static unsigned long lastPrint = 0;
  if(millis() - lastPrint >= 1000) {
    printMotorStatus();
    lastPrint = millis();
  }
  
  if(Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    // === LIMIT SWITCH TEST ===
    if(cmd == "TEST") {
      testLimitSwitches();
    }
    // WEB API COMMANDS
    else if(cmd.startsWith("M")) {
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
    else if(cmd.startsWith("D")) {
      int64_t delta = cmd.substring(1).toInt();
      int64_t targetPos = encoder.getCount() + delta;
      moveToPosition(targetPos, 250);
    }
    else if(cmd.startsWith("P")) {
      int64_t pos = cmd.substring(1).toInt();
      moveToPosition(pos, 250);
    }
    else if(cmd == "COMP") {
      compensation.printStatus();
    }
    else if(cmd == "1") {
      if(digitalRead(LIMIT_SWITCH_12) == HIGH) {
        Serial.println("⚠️ At Limit 12 - cannot move CW!");
      } else {
        setSpeedLevel(DIR_CLOCKWISE, SPEED_SLOW);
        Serial.println("✓ CW SLOW");
      }
    }
    else if(cmd == "2") {
      if(digitalRead(LIMIT_SWITCH_12) == HIGH) {
        Serial.println("⚠️ At Limit 12 - cannot move CW!");
      } else {
        setSpeedLevel(DIR_CLOCKWISE, SPEED_MEDIUM);
        Serial.println("✓ CW MEDIUM");
      }
    }
    else if(cmd == "3") {
      if(digitalRead(LIMIT_SWITCH_12) == HIGH) {
        Serial.println("⚠️ At Limit 12 - cannot move CW!");
      } else {
        setSpeedLevel(DIR_CLOCKWISE, SPEED_FAST);
        Serial.println("✓ CW FAST");
      }
    }
    else if(cmd == "4") {
      if(digitalRead(LIMIT_SWITCH_14) == HIGH) {
        Serial.println("⚠️ At Limit 14 - cannot move CCW!");
      } else {
        setSpeedLevel(DIR_COUNTER_CLOCKWISE, SPEED_SLOW);
        Serial.println("✓ CCW SLOW");
      }
    }
    else {
      Serial.println("✗ Unknown command!");
    }
  }
}
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
#define MIN_STARTUP_SPEED 195

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


// ===== THÊM SAU STRUCT CompensationSystem =====
struct PIDController {
  // ===== PID CORE PARAMETERS =====
  float Kp = 2.0;
  float Ki = 0.01;
  float Kd = 10.0;
  
  int min_output = MIN_STARTUP_SPEED;
  int max_output = 255;
  
  float integral_limit = 100.0;
  float d_limit = 50.0;
  int64_t deadband = 100;
  // ===== PID STATE VARIABLES =====
  float error_integral = 0;
  int64_t last_error = 0;
  unsigned long last_time = 0;
  bool first_call = true;
  
  // ===== COMPENSATION LEARNING (từ Trapezoidal) =====
  float overshootPerRev = 0;
  int crawlSpeedBase = 0;
  int moveCount = 0;
  bool motorIsWarm = false;
  
  float overshootHistory[5] = {0};
  int historyIndex = 0;
  int historyCount = 0;
  
  Direction lastDirection = DIR_CLOCKWISE;
  
  // ===== DECELERATION ZONES =====
  int64_t decel_zone_start = 400;
  int64_t crawl_zone_start = 150;
  int64_t stop_zone_start = 280;
  
  // ===== HỌC TỪ LỊCH SỬ =====
  void addMeasurement(int64_t finalError, int64_t totalDistance, Direction dir) {
    moveCount++;
    lastDirection = dir;
    
    if(moveCount > 2) motorIsWarm = true;
    
    float revolutions = (float)totalDistance / TOTAL_PULSES_PER_REV;
    if(revolutions < 0.1) return;
    
    float errorPerRev = (float)finalError / revolutions;
    
    if(abs(errorPerRev) > 5.0) {
      overshootHistory[historyIndex] = errorPerRev;
      historyIndex = (historyIndex + 1) % 5;
      if(historyCount < 5) historyCount++;
      
      float sum = 0;
      for(int i = 0; i < historyCount; i++) {
        sum += overshootHistory[i];
      }
      overshootPerRev = sum / historyCount;
      
      Serial.printf("🎓 Learned overshoot/rev: %.1f pulses (from %d moves)\n", 
                    overshootPerRev, historyCount);
    }
    
    bool isUndershoot = (dir == DIR_CLOCKWISE && finalError < 0) || 
                        (dir == DIR_COUNTER_CLOCKWISE && finalError > 0);
    
    if(isUndershoot && abs(errorPerRev) > 10.0) {
      crawlSpeedBase += 5;
      Serial.println("⚙️ Undershoot detected → crawl speed +5");
    } else if(!isUndershoot && abs(errorPerRev) > 15.0) {
      crawlSpeedBase -= 2;
      Serial.println("⚙️ Overshoot detected → crawl speed -2");
    }
    
    crawlSpeedBase = constrain(crawlSpeedBase, -20, 30);
  }
  
  // ===== DỰ ĐOÁN OVERSHOOT =====
  int64_t getPredictedOvershoot(int64_t totalDistance) {
    if(historyCount < 2) return 0;
    
    float revolutions = (float)totalDistance / TOTAL_PULSES_PER_REV;
    return (int64_t)(overshootPerRev * revolutions);
  }
  
  // ===== ADAPTIVE CRAWL SPEED =====
  int getAdaptiveCrawlSpeed(int64_t totalDistance) {
    int baseCrawl = MIN_STARTUP_SPEED - 20;  // 175
    
    int warmBoost = motorIsWarm ? 5 : 0;
    
    int distanceBoost = 0;
    if(totalDistance > 5000) {
      distanceBoost = 10;
    } else if(totalDistance < 1000) {
      distanceBoost = -5;
    }
    
    int adaptiveCrawl = baseCrawl + crawlSpeedBase + warmBoost + distanceBoost;
    return constrain(adaptiveCrawl, MIN_STARTUP_SPEED - 30, MIN_STARTUP_SPEED + 10);
  }
  
  // ===== PID COMPUTE với COMPENSATION =====
  int compute(int64_t setpoint, int64_t current_position, int64_t totalDistance, Direction dir) {
    unsigned long now = millis();
    
    // ===== FIRST CALL - BÙ QUÁN TÍNH =====
    if(first_call) {
      last_time = now;
      
      int64_t raw_error = setpoint - current_position;
      
      if(historyCount >= 2 && totalDistance > 500) {
        int64_t predictedOvershoot = getPredictedOvershoot(totalDistance);
        
        if(dir == DIR_CLOCKWISE) {
          raw_error -= predictedOvershoot;
        } else {
          raw_error += predictedOvershoot;
        }
        
        Serial.printf("🎯 Initial compensation: %ld pulses (%.1f/rev × %.2f revs)\n",
                     (long)predictedOvershoot, overshootPerRev,
                     (float)totalDistance / TOTAL_PULSES_PER_REV);
      }
      
      last_error = raw_error;
      first_call = false;
      
      float P = Kp * raw_error;
      int pwm = constrain((int)abs(P), min_output, max_output);
      
      Serial.printf("🚀 PID Start: P=%.1f → PWM=%d\n", P, pwm);
      return pwm;
    }
    
    // ===== NORMAL PID LOOP =====
    float dt = (now - last_time) / 1000.0;
    if(dt <= 0 || dt > 1.0) dt = 0.01;
    
    int64_t error = setpoint - current_position;
    
    if(abs(error) <= deadband) {
      Serial.println("🎯 Within deadband → STOP");
      return 0;
    }
    
    // ===== P TERM =====
    float P = Kp * error;
    
    // ===== I TERM với ANTI-WINDUP =====
    error_integral += error * dt;
    error_integral = constrain(error_integral, -integral_limit, integral_limit);
    float I = Ki * error_integral;
    
    // ===== D TERM với LIMITING =====
    float error_derivative = (error - last_error) / dt;
    float D = Kd * error_derivative;
    D = constrain(D, -d_limit, d_limit);
    
    // ===== TOTAL PID OUTPUT =====
    float pid_output = P + I + D;
    int pwm = (int)abs(pid_output);
    
    // ===== DECELERATION ZONES với ADAPTIVE CRAWL =====
    int64_t abs_error = abs(error);
    
    if(abs_error > decel_zone_start) {
      pwm = constrain(pwm, min_output, max_output);
    }
    else if(abs_error > crawl_zone_start) {
      float progress = (float)(abs_error - crawl_zone_start) / (decel_zone_start - crawl_zone_start);
      int decelSpeed = min_output + (int)(progress * (max_output - min_output));
      pwm = max(pwm, decelSpeed);
    }
    else if(abs_error > stop_zone_start) {
      int adaptiveCrawl = getAdaptiveCrawlSpeed(totalDistance);
      pwm = max(pwm, adaptiveCrawl);
      
      if(millis() % 500 == 0) {
        Serial.printf("🐌 Crawl: PWM=%d (adaptive=%d)\n", pwm, adaptiveCrawl);
      }
    }
    else {
      int stopSpeed = min_output - 45;  // Giảm từ 160 xuống 150 PWM
      
      // Học overshoot → giảm thêm
      if(historyCount >= 2) {
        stopSpeed -= 15;  // 135 PWM sau 2 moves
      }
      
      pwm = max(pwm, stopSpeed);
      
      if(millis() % 500 == 0) {
        Serial.printf("🛑 Stop zone: PWM=%d\n", pwm);
      }
    }
    
    if(pwm < min_output) pwm = min_output;
    pwm = constrain(pwm, min_output, max_output);
    
    last_error = error;
    last_time = now;
    
    return pwm;
  }
  
  // ===== RESET =====
  void resetPID() {
    error_integral = 0;
    last_error = 0;
    last_time = 0;
    first_call = true;
    
    Serial.printf("✅ PID reset (I=%.1f → 0)\n", error_integral);
  }
  
  // ===== TUNE =====
  void tune(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    Serial.printf("✅ PID tuned: Kp=%.2f Ki=%.3f Kd=%.1f\n", Kp, Ki, Kd);
  }
  
  // ===== PRINT STATUS =====
  void printStatus() {
    Serial.println("\n╔═══ PID + COMPENSATION STATUS ═══╗");
    Serial.printf("║ Moves: %d | Warm: %s\n", moveCount, motorIsWarm ? "YES" : "NO");
    Serial.printf("║ Overshoot/rev: %.1f pulses (from %d moves)\n", 
                 overshootPerRev, historyCount);
    Serial.printf("║ Crawl adjustment: %+d PWM\n", crawlSpeedBase);
    Serial.printf("║ PID gains: Kp=%.2f Ki=%.3f Kd=%.1f\n", Kp, Ki, Kd);
    Serial.printf("║ Integral: %.1f (limit=%.1f)\n", error_integral, integral_limit);
    Serial.println("╚══════════════════════════════════╝");
  }
};


CompensationSystem compensation;
PIDController pid;

ESP32Encoder encoder;
int currentSpeed = 0;
Direction currentDirection = DIR_STOP;
unsigned long lastSpeedCheck = 0;
int64_t lastEncoderPosition = 0;
float motorRPM = 0;

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
  Serial.printf("Pos:%ld | Angle:%.1f° | Rev:%.2f | RPM:%.1f | Speed:%d | Dir:%s\n",
    (long)encoder.getCount(), getAngleDegrees(), getRevolutions(), 
    motorRPM, currentSpeed,
    currentDirection == DIR_CLOCKWISE ? "CW" : 
    currentDirection == DIR_COUNTER_CLOCKWISE ? "CCW" : "STOP");
}

// ===== FUNCTION TEST - CHỈ TRONG NÀY MỚI KIỂM TRA LIMIT SWITCH =====
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
// ===== MOVE TO POSITION USING PID CONTROLLER =====
void moveToPositionPID(int64_t targetPulses, int maxSpeed = 250) {
  if(maxSpeed < MIN_STARTUP_SPEED) {
    maxSpeed = MIN_STARTUP_SPEED;
  }
  
  pid.resetPID();
  
  int64_t startPos = encoder.getCount();
  int64_t totalDistance = abs(targetPulses - startPos);
  
  Serial.printf("\n╔═══ PID MOVE #%d ═══╗\n", pid.moveCount + 1);
  Serial.printf("║ Target: %ld | Distance: %ld pulses (%.2f revs)\n",
    (long)targetPulses, (long)totalDistance, 
    (float)totalDistance / TOTAL_PULSES_PER_REV);
  
  if(totalDistance < 5) {
    Serial.println("║ ✅ Already at target!");
    Serial.println("╚═══════════════════════╝");
    return;
  }
  
  Direction dir = (targetPulses > startPos) ? DIR_CLOCKWISE : DIR_COUNTER_CLOCKWISE;
  
  // ✅ HIỂN THỊ COMPENSATION
  if(pid.historyCount >= 2 && totalDistance > 500) {
    int64_t predictedOvershoot = pid.getPredictedOvershoot(totalDistance);
    Serial.printf("║ 🎯 Predicted overshoot: %ld pulses\n", (long)predictedOvershoot);
  }
  
  int adaptiveCrawl = pid.getAdaptiveCrawlSpeed(totalDistance);
  Serial.printf("║ Adaptive crawl: %d PWM (base=%d)\n", adaptiveCrawl, pid.crawlSpeedBase);
  
  Serial.printf("║ Direction: %s | Warm: %s\n",
    dir == DIR_CLOCKWISE ? "CW" : "CCW",
    pid.motorIsWarm ? "YES" : "NO");
  Serial.println("╚═══════════════════════╝");
  
  setMotorDirection(dir);
  
  unsigned long startTime = millis();
  unsigned long lastProgressTime = millis();
  int64_t lastProgressPos = startPos;
  
  while(millis() - startTime < 30000) {
    readEncoderSpeed();
    
    int64_t currentPos = encoder.getCount();
    int64_t error = targetPulses - currentPos;
    
    // ✅ PID COMPUTE với COMPENSATION
    int pwm = pid.compute(targetPulses, currentPos, totalDistance, dir);
    
    if(pwm == 0) {
      break;  // Deadband reached
    }
    
    setMotorSpeed(pwm);
    
    // Stuck detection
    if(millis() - lastProgressTime > 2000) {
      int64_t moved = abs(currentPos - lastProgressPos);
      if(moved < 10) {
        Serial.println("⚠️ STUCK detected! Breaking...");
        break;
      }
      lastProgressTime = millis();
      lastProgressPos = currentPos;
    }
    
    // Target reached check
    if(dir == DIR_CLOCKWISE && currentPos >= targetPulses) break;
    if(dir == DIR_COUNTER_CLOCKWISE && currentPos <= targetPulses) break;
    
    delay(5);
  }
  
  // ===== GENTLE BRAKE =====
  motorStop();
  delay(50);
  
  Direction brakeDir = (dir == DIR_CLOCKWISE) ? DIR_COUNTER_CLOCKWISE : DIR_CLOCKWISE;
  setMotorDirection(brakeDir);
  setMotorSpeed(160);
  delay(30);
  motorStop();
  
  delay(800);
  
  // ===== RESULT & LEARNING =====
  int64_t finalPos = encoder.getCount();
  int64_t finalError = finalPos - targetPulses;
  
  // ✅ HỌC TỪ KẾT QUẢ
  pid.addMeasurement(finalError, totalDistance, dir);
  
  Serial.println("\n╔═══ PID RESULT ═══╗");
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
  } else if(abs(finalError) <= 50) {
    Serial.println("║ ⚠️  ACCEPTABLE");
  } else {
    Serial.println("║ ❌ POOR - Learning...");
  }
  Serial.println("╚═══════════════════╝");
  
  pid.printStatus();
  printDetailedStatus();
}
// ===== MOVE TO POSITION - KHÔNG KIỂM TRA LIMIT SWITCH =====
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
      
      int minAllowed = MIN_STARTUP_SPEED - 25;
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

// WEB API FUNCTIONS
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
// ===== PID API FUNCTIONS =====

void API_moveByDegreesPID(float degrees) {
  int64_t pulses = (int64_t)((degrees / 360.0) * TOTAL_PULSES_PER_REV);
  int64_t targetPos = encoder.getCount() + pulses;
  
  Serial.printf("\n🌐 API (PID): Move by %.2f degrees (%ld pulses)\n", degrees, (long)pulses);
  moveToPositionPID(targetPos, 250);
}

void API_moveToAbsoluteDegreesPID(float degrees) {
  while(degrees < 0) degrees += 360;
  while(degrees >= 360) degrees -= 360;
  
  float currentAngle = getAngleDegrees();
  float angleDiff = degrees - currentAngle;
  
  if(angleDiff > 180) angleDiff -= 360;
  if(angleDiff < -180) angleDiff += 360;
  
  int64_t pulses = (int64_t)((angleDiff / 360.0) * TOTAL_PULSES_PER_REV);
  int64_t targetPos = encoder.getCount() + pulses;
  
  Serial.printf("\n🌐 API (PID): Move to %.2f° (from %.2f°, delta %.2f°)\n", 
    degrees, currentAngle, angleDiff);
  moveToPositionPID(targetPos, 250);
}

void API_moveRevolutionsPID(float revolutions) {
  int64_t pulses = (int64_t)(revolutions * TOTAL_PULSES_PER_REV);
  int64_t targetPos = encoder.getCount() + pulses;
  
  Serial.printf("\n🌐 API (PID): Move %.2f revolutions (%ld pulses)\n", 
    revolutions, (long)pulses);
  moveToPositionPID(targetPos, 250);
}

void API_tunePID(float kp, float ki, float kd) {
  Serial.printf("\n🌐 API: Tune PID (Kp=%.2f Ki=%.3f Kd=%.1f)\n", kp, ki, kd);
  pid.tune(kp, ki, kd);
}

void API_resetPID() {
  Serial.println("\n🌐 API: Reset PID");
  pid.resetPID();
  pid.moveCount = 0;
  pid.historyCount = 0;
  pid.overshootPerRev = 0;
  pid.crawlSpeedBase = 0;
  pid.motorIsWarm = false;
}
String API_getPIDStatus() {
  String json = "{";
  json += "\"kp\":" + String(pid.Kp, 2) + ",";
  json += "\"ki\":" + String(pid.Ki, 3) + ",";
  json += "\"kd\":" + String(pid.Kd, 1) + ",";
  json += "\"moves\":" + String(pid.moveCount) + ",";
  json += "\"overshoot\":" + String(pid.overshootPerRev, 1) + ",";
  json += "\"integral\":" + String(pid.error_integral, 1) + ",";
  json += "\"deadband\":" + String((long)pid.deadband);
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
  
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RL_EN, PWM_CHANNEL);
  
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachFullQuad(ENCODER_C1, ENCODER_C2);
  encoder.setFilter(1023);
  encoder.clearCount();
  
  motorStop();
  
  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║   ESP32-S3 Motor Control - PID + LIMITS       ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  Serial.println("\n🌐 OLD Commands (Trapezoidal):");
  Serial.println("  M90      - Move by 90°");
  Serial.println("  A90      - Move to absolute 90°");
  Serial.println("  R1.5     - Move 1.5 revolutions");
  
  Serial.println("\n🎯 NEW Commands (PID):");
  Serial.println("  PM90     - PID Move by 90°");
  Serial.println("  PA90     - PID Move to absolute 90°");
  Serial.println("  PR1.5    - PID Move 1.5 revolutions");
  Serial.println("  TUNE <Kp> <Ki> <Kd> - Tune PID gains");
  Serial.println("           Example: TUNE 0.8 0.01 50");
  Serial.println("  PIDINFO  - Show PID status");
  Serial.println("  PIDRESET - Reset PID");
  Serial.println("  DEADBAND <n> - Set deadband (pulses)");
  
  Serial.println("\n📊 Other Commands:");
  Serial.println("  TEST     - Run limit switch test");
  Serial.println("  POS      - Get position (JSON)");
  Serial.println("  STATUS   - Get full status (JSON)");
  Serial.println("  PIDSTATUS- Get PID status (JSON)");
  Serial.println("  RESET    - Reset position");
  Serial.println("  STOP     - Emergency stop");
  
  Serial.printf("\nPPR: %d | Motor: %d RPM\n", TOTAL_PULSES_PER_REV, MOTOR_RPM);
  Serial.println("\n>>> Ready! <<<\n");
}
void loop() {
  readEncoderSpeed();
  
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
    
  // ===== PID + COMPENSATION COMMANDS =====
  else if(cmd == "PM" || cmd.startsWith("PM")) {
    // PID move by degrees
    float degrees = cmd.substring(2).toFloat();
    if(degrees == 0) degrees = 90;
    int64_t targetPulses = encoder.getCount() + (int64_t)((degrees / 360.0) * TOTAL_PULSES_PER_REV);
    Serial.printf("📍 PID Moving %.1f degrees to %ld\n", degrees, (long)targetPulses);
    moveToPositionPID(targetPulses);
  }
  else if(cmd == "PA" || cmd.startsWith("PA")) {
    // PID move to absolute position
    float degrees = cmd.substring(2).toFloat();
    int64_t targetPulses = (int64_t)((degrees / 360.0) * TOTAL_PULSES_PER_REV);
    Serial.printf("📍 PID Moving to absolute %.1f° (%ld pulses)\n", degrees, (long)targetPulses);
    moveToPositionPID(targetPulses);
  }
  else if(cmd == "PR" || cmd.startsWith("PR")) {
    // PID move by revolutions
    float revs = cmd.substring(2).toFloat();
    if(revs == 0) revs = 1.0;
    int64_t targetPulses = encoder.getCount() + (int64_t)(revs * TOTAL_PULSES_PER_REV);
    Serial.printf("📍 PID Moving %.2f revolutions to %ld\n", revs, (long)targetPulses);
    moveToPositionPID(targetPulses);
  }
  else if(cmd == "PIDINFO") {
    pid.printStatus();
  }
  else if(cmd == "PIDRESET") {
    pid.resetPID();
    pid.moveCount = 0;
    pid.historyCount = 0;
    pid.overshootPerRev = 0;
    pid.crawlSpeedBase = 0;
    pid.motorIsWarm = false;
    Serial.println("✅ PID + Compensation fully reset!");
  }
  else if(cmd == "PIDLEARN") {
    // Reset learning, giữ PID gains
    pid.moveCount = 0;
    pid.historyCount = 0;
    pid.overshootPerRev = 0;
    pid.crawlSpeedBase = 0;
    pid.motorIsWarm = false;
    Serial.println("✅ Compensation learning reset!");
  }
  else if(cmd.startsWith("TUNE")) {
    // TUNE Kp Ki Kd
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    if(secondSpace > 0) {
      pid.Kp = cmd.substring(firstSpace + 1, secondSpace).toFloat();
      pid.Ki = cmd.substring(secondSpace + 1).toFloat();
      
      int thirdSpace = cmd.indexOf(' ', secondSpace + 1);
      if(thirdSpace > 0) {
        pid.Kd = cmd.substring(thirdSpace + 1).toFloat();
      }
      
      Serial.printf("✅ PID tuned: Kp=%.2f Ki=%.3f Kd=%.1f\n", pid.Kp, pid.Ki, pid.Kd);
    }
  }
    
    // ===== OLD COMMANDS (GIỮ NGUYÊN) =====
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
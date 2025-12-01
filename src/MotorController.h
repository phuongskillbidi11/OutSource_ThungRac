#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include <ESP32Encoder.h>

// Thêm cờ kiểm tra motors đã sẵn sàng
extern bool motorsReady;

enum Direction { 
  DIR_STOP = 0, 
  DIR_CLOCKWISE = 1, 
  DIR_COUNTER_CLOCKWISE = 2 
};

enum SpeedLevel { 
  SPEED_STOP = 0, 
  SPEED_1 = 1, 
  SPEED_2 = 2, 
  SPEED_3 = 3, 
  SPEED_4 = 4 
};
#define MIN_STARTUP_SPEED 195
// ===== PID CONTROLLER =====
struct PIDController {
  // PID Core Parameters
  float Kp = 2.0;
  float Ki = 0.01;
  float Kd = 10.0;
  
  int min_output = MIN_STARTUP_SPEED; 
  int max_output = 255;
  
  float integral_limit = 100.0;
  float d_limit = 50.0;
  int64_t deadband = 100;
  
  // PID State
  float error_integral = 0;
  int64_t last_error = 0;
  unsigned long last_time = 0;
  bool first_call = true;
  
  // Learning
  float overshootPerRev = 0;
  int crawlSpeedBase = 0;
  int moveCount = 0;
  bool motorIsWarm = false;
  
  float overshootHistory[5] = {0};
  int historyIndex = 0;
  int historyCount = 0;
  
  Direction lastDirection = DIR_CLOCKWISE;
  
  // Zones
  int64_t decel_zone_start = 400;
  int64_t crawl_zone_start = 150;
  int64_t stop_zone_start = 280;
  
  void addMeasurement(int64_t finalError, int64_t totalDistance, Direction dir) {
    moveCount++;
    lastDirection = dir;
    
    if(moveCount > 2) motorIsWarm = true;
    
    float revolutions = (float)totalDistance / 1356.0;
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
      
      Serial.printf("🎯 PID learned overshoot/rev: %.1f pulses\n", overshootPerRev);
    }
    
    bool isUndershoot = (dir == DIR_CLOCKWISE && finalError < 0) || 
                        (dir == DIR_COUNTER_CLOCKWISE && finalError > 0);
    
    if(isUndershoot && abs(errorPerRev) > 10.0) {
      crawlSpeedBase += 5;
    } else if(!isUndershoot && abs(errorPerRev) > 15.0) {
      crawlSpeedBase -= 2;
    }
    
    crawlSpeedBase = constrain(crawlSpeedBase, -20, 30);
  }
  
  int64_t getPredictedOvershoot(int64_t totalDistance, int ppr) {
    if(historyCount < 2) return 0;
    float revolutions = (float)totalDistance / ppr;
    return (int64_t)(overshootPerRev * revolutions);
  }
  
  int getAdaptiveCrawlSpeed(int64_t totalDistance) {
    int baseCrawl = min_output - 20;
    int warmBoost = motorIsWarm ? 5 : 0;
    
    int distanceBoost = 0;
    if(totalDistance > 5000) distanceBoost = 10;
    else if(totalDistance < 1000) distanceBoost = -5;
    
    int adaptiveCrawl = baseCrawl + crawlSpeedBase + warmBoost + distanceBoost;
    return constrain(adaptiveCrawl, min_output - 30, min_output + 10);
  }
  
  int compute(int64_t setpoint, int64_t current_position, int64_t totalDistance, Direction dir, int ppr) {
    unsigned long now = millis();
    
    if(first_call) {
      last_time = now;
      
      int64_t raw_error = setpoint - current_position;
      
      if(historyCount >= 2 && totalDistance > 500) {
        int64_t predictedOvershoot = getPredictedOvershoot(totalDistance, ppr);
        
        if(dir == DIR_CLOCKWISE) {
          raw_error -= predictedOvershoot;
        } else {
          raw_error += predictedOvershoot;
        }
        
        Serial.printf("🎯 PID compensation: %ld pulses\n", (long)predictedOvershoot);
      }
      
      last_error = raw_error;
      first_call = false;
      
      float P = Kp * raw_error;
      int pwm = constrain((int)abs(P), min_output, max_output);
      
      Serial.printf("🚀 PID Start: P=%.1f → PWM=%d\n", P, pwm);
      return pwm;
    }
    
    float dt = (now - last_time) / 1000.0;
    if(dt <= 0 || dt > 1.0) dt = 0.01;
    
    int64_t error = setpoint - current_position;
    
    if(abs(error) <= deadband) {
      Serial.println("🎯 PID: Within deadband → STOP");
      return 0;
    }
    
    float P = Kp * error;
    error_integral += error * dt;
    error_integral = constrain(error_integral, -integral_limit, integral_limit);
    float I = Ki * error_integral;
    
    float error_derivative = (error - last_error) / dt;
    float D = Kd * error_derivative;
    D = constrain(D, -d_limit, d_limit);
    
    float pid_output = P + I + D;
    int pwm = (int)abs(pid_output);
    
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
    }
    else {
      int stopSpeed = min_output - 45;
      if(historyCount >= 2) stopSpeed -= 15;
      pwm = max(pwm, stopSpeed);
    }
    
    pwm = constrain(pwm, min_output, max_output);
    
    last_error = error;
    last_time = now;
    
    return pwm;
  }
  
  void reset() {
    error_integral = 0;
    last_error = 0;
    last_time = 0;
    first_call = true;
  }
  
  void resetLearning() {
    moveCount = 0;
    historyCount = 0;
    overshootPerRev = 0;
    crawlSpeedBase = 0;
    motorIsWarm = false;
    for(int i = 0; i < 5; i++) overshootHistory[i] = 0;
    historyIndex = 0;
  }
  
  void tune(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    Serial.printf("✅ PID tuned: Kp=%.2f Ki=%.3f Kd=%.1f\n", Kp, Ki, Kd);
  }
  
  void printStatus() {
    Serial.println("\n╔═══ PID STATUS ═══╗");
    Serial.printf("║ Moves: %d | Warm: %s\n", moveCount, motorIsWarm ? "YES" : "NO");
    Serial.printf("║ Overshoot/rev: %.1f pulses\n", overshootPerRev);
    Serial.printf("║ Crawl adj: %+d PWM\n", crawlSpeedBase);
    Serial.printf("║ Gains: Kp=%.2f Ki=%.3f Kd=%.1f\n", Kp, Ki, Kd);
    Serial.printf("║ Integral: %.1f\n", error_integral);
    Serial.println("╚══════════════════╝");
  }
};

// ===== MOTOR STRUCT =====
struct Motor {
  int id = 0;
  String name = "";
  bool initialized = false;  // ← THÊM CỜ NÀY
  
  int rl_en = 0, r_pwm = 0, l_pwm = 0, enc_c1 = 0, enc_c2 = 0;
  
  ESP32Encoder encoder;
  int pulsesPerRev = 1356;
  float degreesPerPulse = 0.265;
  
  Direction currentDir = DIR_STOP;
  SpeedLevel currentSpeed = SPEED_STOP;
  bool jogMode = false;
  
  bool softLimitsEnabled = false;
  int64_t softLimitMin = -99999;
  int64_t softLimitMax = 99999;
  
  PIDController pid;
  float kp = 1.0, ki = 0.0, kd = 0.0;
  
  unsigned long lastSpeedCheck = 0;
  int64_t lastEncoderPosition = 0;
  float motorRPM = 0.0;
};

Motor motors[10];
int motorCount = 0;
bool motorsReady = false;  

int speedPWM[5] = {0, 64, 128, 192, 255};

// ===== SETUP MOTOR =====
void motorSetup(int index, int _id, String _name, 
                int _rl_en, int _r_pwm, int _l_pwm, 
                int _enc_c1, int _enc_c2, 
                int _ppr, float _kp, float _ki, float _kd,
                int64_t _soft_min, int64_t _soft_max) {
  
  if (index >= 10) {
    Serial.printf("ERROR: Motor index %d >= 10!\n", index);
    return;
  }
  
  Motor &m = motors[index];
  
  m.id = _id;
  m.name = _name;
  m.rl_en = _rl_en;
  m.r_pwm = _r_pwm;
  m.l_pwm = _l_pwm;
  m.enc_c1 = _enc_c1;
  m.enc_c2 = _enc_c2;
  m.pulsesPerRev = _ppr;
  m.degreesPerPulse = 360.0 / _ppr;
  
  m.kp = _kp;
  m.ki = _ki;
  m.kd = _kd;
  m.pid.tune(_kp, _ki, _kd);
  
  m.softLimitMin = _soft_min;
  m.softLimitMax = _soft_max;
  m.softLimitsEnabled = false;
  
  m.currentDir = DIR_STOP;
  m.currentSpeed = SPEED_STOP;
  m.jogMode = false;
  
  m.lastSpeedCheck = millis();
  m.lastEncoderPosition = 0;
  m.motorRPM = 0;
  
  pinMode(m.rl_en, OUTPUT);
  pinMode(m.r_pwm, OUTPUT);
  pinMode(m.l_pwm, OUTPUT);
  digitalWrite(m.rl_en, HIGH);
  
  int r_channel = 0 + (index * 2);
  int l_channel = 1 + (index * 2);
  
  ledcSetup(r_channel, 20000, 8);
  ledcSetup(l_channel, 20000, 8);
  ledcAttachPin(m.r_pwm, r_channel);
  ledcAttachPin(m.l_pwm, l_channel);
  ledcWrite(r_channel, 0);
  ledcWrite(l_channel, 0);
  
  delay(10);
  
  ESP32Encoder::useInternalWeakPullResistors = UP;
  m.encoder.attachFullQuad(m.enc_c1, m.enc_c2);
  m.encoder.setFilter(1023);
  m.encoder.clearCount();
  
  delay(10);
  
  m.initialized = true;  // ← ĐÁN DẤU MOTOR ĐÃ SẴN SÀNG
  
  Serial.printf("Motor %d (%s) ready - Pins: EN=%d R=%d L=%d ENC=%d/%d\n", 
    m.id, m.name.c_str(), m.rl_en, m.r_pwm, m.l_pwm, m.enc_c1, m.enc_c2);
}

// ===== MOTOR CONTROL =====
void motorSetPWM(int index, Direction dir, int pwm) {
  if(index >= motorCount || !motors[index].initialized) return;
  
  Motor &m = motors[index];
  
  m.currentDir = dir;
  pwm = constrain(pwm, 0, 255);
  
  int r_channel = 0 + (index * 2);
  int l_channel = 1 + (index * 2);
  
  if (dir == DIR_CLOCKWISE) {
    ledcWrite(r_channel, pwm);
    ledcWrite(l_channel, 0);
  } else if (dir == DIR_COUNTER_CLOCKWISE) {
    ledcWrite(r_channel, 0);
    ledcWrite(l_channel, pwm);
  } else {  // ← THÊM ELSE NÀY
    ledcWrite(r_channel, 0);
    ledcWrite(l_channel, 0);
  }
}

void motorSetSpeed(int index, Direction dir, SpeedLevel spd) {
  if(index >= motorCount || !motors[index].initialized) return;
  
  Motor &m = motors[index];
  
  m.currentDir = dir;
  m.currentSpeed = spd;
  
  int pwm = speedPWM[spd];
  
  int r_channel = 0 + (index * 2);
  int l_channel = 1 + (index * 2);
  
  if (dir == DIR_CLOCKWISE) {
    ledcWrite(r_channel, pwm);
    ledcWrite(l_channel, 0);
  } else if (dir == DIR_COUNTER_CLOCKWISE) {
    ledcWrite(r_channel, 0);
    ledcWrite(l_channel, pwm);
  } else {
    ledcWrite(r_channel, 0);
    ledcWrite(l_channel, 0);
  }
}

void motorStop(int index) {
  if(index >= motorCount || !motors[index].initialized) return;
  motorSetSpeed(index, DIR_STOP, SPEED_STOP);
  motors[index].jogMode = false;
}

void motorStartJog(int index, Direction dir, SpeedLevel spd) {
  if(index >= motorCount || !motors[index].initialized) return;
  motors[index].jogMode = true;
  motorSetSpeed(index, dir, spd);
}

void motorStopJog(int index) {
  motorStop(index);
}

float motorGetAngle(int index) {
  if(index >= motorCount || !motors[index].initialized) return 0.0;
  Motor &m = motors[index];
  return m.encoder.getCount() * m.degreesPerPulse;
}

int64_t motorGetPosition(int index) {
  if(index >= motorCount || !motors[index].initialized) return 0;
  return motors[index].encoder.getCount();
}

void motorZeroPosition(int index) {
  if(index >= motorCount || !motors[index].initialized) return;
  motors[index].encoder.clearCount();
  motors[index].pid.resetLearning();
  Serial.printf("Motor %d zeroed\n", motors[index].id);
}

void motorCheckSoftLimits(int index) {
  if(index >= motorCount || !motors[index].initialized) return;
  Motor &m = motors[index];
  
  if (!m.softLimitsEnabled || !m.jogMode) return;
  
  int64_t pos = m.encoder.getCount();
  
  if ((m.currentDir == DIR_CLOCKWISE && pos >= m.softLimitMax) ||
      (m.currentDir == DIR_COUNTER_CLOCKWISE && pos <= m.softLimitMin)) {
    Serial.printf("Motor %d hit soft limit!\n", m.id);
    motorStop(index);
  }
}

// ===== PID MOVE =====
void motorMovePID(int index, int64_t targetPulses) {
  if(index >= motorCount || !motors[index].initialized) return;
  
  Motor &m = motors[index];
  
  m.pid.reset();
  
  int64_t startPos = m.encoder.getCount();
  int64_t totalDistance = abs(targetPulses - startPos);
  
  Serial.printf("\n╔═══ M%d PID MOVE #%d ═══╗\n", m.id, m.pid.moveCount + 1);
  Serial.printf("║ Target: %ld | Distance: %ld pulses\n",
    (long)targetPulses, (long)totalDistance);
  
  if(totalDistance < 5) {
    Serial.println("║ ✅ Already at target!");
    Serial.println("╚════════════════════╝");
    return;
  }
  
  Direction dir = (targetPulses > startPos) ? DIR_CLOCKWISE : DIR_COUNTER_CLOCKWISE;
  
  Serial.printf("║ Direction: %s\n", dir == DIR_CLOCKWISE ? "CW" : "CCW");
  Serial.println("╚════════════════════╝");
  
  motorSetPWM(index, dir, 0);
  
  unsigned long startTime = millis();
  unsigned long lastProgressTime = millis();
  int64_t lastProgressPos = startPos;
  
  while(millis() - startTime < 30000) {
    yield();
    
    int64_t currentPos = m.encoder.getCount();
    
    int pwm = m.pid.compute(targetPulses, currentPos, totalDistance, dir, m.pulsesPerRev);
    
    if(pwm == 0) break;
    
    motorSetPWM(index, dir, pwm);
    
    if(millis() - lastProgressTime > 2000) {
      int64_t moved = abs(currentPos - lastProgressPos);
      if(moved < 10) {
        Serial.println("⚠️ STUCK!");
        break;
      }
      lastProgressTime = millis();
      lastProgressPos = currentPos;
    }
    
    if(dir == DIR_CLOCKWISE && currentPos >= targetPulses) break;
    if(dir == DIR_COUNTER_CLOCKWISE && currentPos <= targetPulses) break;
    
    delay(5);
  }
  
  motorStop(index);
  delay(50);
  
  Direction brakeDir = (dir == DIR_CLOCKWISE) ? DIR_COUNTER_CLOCKWISE : DIR_CLOCKWISE;
  motorSetPWM(index, brakeDir, 160);
  delay(30);
  motorStop(index);
  
  delay(800);
  
  int64_t finalPos = m.encoder.getCount();
  int64_t finalError = finalPos - targetPulses;
  
  m.pid.addMeasurement(finalError, totalDistance, dir);
  
  Serial.println("\n╔═══ RESULT ═══╗");
  Serial.printf("║ Target: %ld\n", (long)targetPulses);
  Serial.printf("║ Actual: %ld\n", (long)finalPos);
  Serial.printf("║ Error: %ld pulses\n", (long)finalError);
  
  if(abs(finalError) <= 5) Serial.println("║ ✅ EXCELLENT!");
  else if(abs(finalError) <= 15) Serial.println("║ ✅ GOOD!");
  else if(abs(finalError) <= 50) Serial.println("║ ⚠️ ACCEPTABLE");
  else Serial.println("║ ❌ POOR");
  
  Serial.println("╚═══════════════╝");
    m.lastSpeedCheck = millis();
  m.lastEncoderPosition = m.encoder.getCount();
}

void motorsPrintAllStatus() {
  for (int i = 0; i < motorCount; i++) {
    if(!motors[i].initialized) continue;
    Motor &m = motors[i];
    Serial.printf("[M%d:%s] Pos:%lld Angle:%.2f° Dir:%d Spd:%d\n",
      m.id, m.name.c_str(), m.encoder.getCount(), motorGetAngle(i), 
      m.currentDir, m.currentSpeed);
  }
}

void motorsCheckAllSoftLimits() {
  for (int i = 0; i < motorCount; i++) {
    motorCheckSoftLimits(i);
  }
}

#endif
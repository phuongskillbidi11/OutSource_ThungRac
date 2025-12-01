#include <Arduino.h>
#include <ESP32Encoder.h>

// ===== ĐỊNH NGHĨA CHÂN BTS7960 DRIVER =====
#define RL_EN  4
#define R_PWM  5
#define L_PWM  6

// ===== ĐỊNH NGHĨA CHÂN ENCODER =====
#define ENCODER_C1  35
#define ENCODER_C2  36

// ===== ĐỊNH NGHĨA CHÂN LIMIT SWITCH =====
#define LIMIT_SWITCH_12 12
#define LIMIT_SWITCH_14 14

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
  SPEED_1 = 160,
  SPEED_2 = 190,
  SPEED_3 = 220,
  SPEED_4 = 255
};

enum Direction {
  DIR_STOP = 0,
  DIR_CLOCKWISE = 1,
  DIR_COUNTER_CLOCKWISE = 2
};

// ===== PID CONTROLLER với COMPENSATION =====
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
  
  // PID State Variables
  float error_integral = 0;
  int64_t last_error = 0;
  unsigned long last_time = 0;
  bool first_call = true;
  
  // Compensation Learning
  float overshootPerRev = 0;
  int crawlSpeedBase = 0;
  int moveCount = 0;
  bool motorIsWarm = false;
  
  float overshootHistory[5] = {0};
  int historyIndex = 0;
  int historyCount = 0;
  
  Direction lastDirection = DIR_CLOCKWISE;
  
  // Deceleration Zones
  int64_t decel_zone_start = 400;
  int64_t crawl_zone_start = 150;
  int64_t stop_zone_start = 280;
  
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
      Serial.println("⚙️ Undershoot → crawl +5");
    } else if(!isUndershoot && abs(errorPerRev) > 15.0) {
      crawlSpeedBase -= 2;
      Serial.println("⚙️ Overshoot → crawl -2");
    }
    
    crawlSpeedBase = constrain(crawlSpeedBase, -20, 30);
  }
  
  int64_t getPredictedOvershoot(int64_t totalDistance) {
    if(historyCount < 2) return 0;
    
    float revolutions = (float)totalDistance / TOTAL_PULSES_PER_REV;
    return (int64_t)(overshootPerRev * revolutions);
  }
  
  int getAdaptiveCrawlSpeed(int64_t totalDistance) {
    int baseCrawl = MIN_STARTUP_SPEED - 20;
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
  
  int compute(int64_t setpoint, int64_t current_position, int64_t totalDistance, Direction dir) {
    unsigned long now = millis();
    
    // First Call - Compensation
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
        
        Serial.printf("🎯 Initial compensation: %ld pulses\n", (long)predictedOvershoot);
      }
      
      last_error = raw_error;
      first_call = false;
      
      float P = Kp * raw_error;
      int pwm = constrain((int)abs(P), min_output, max_output);
      
      Serial.printf("🚀 PID Start: P=%.1f → PWM=%d\n", P, pwm);
      return pwm;
    }
    
    // Normal PID Loop
    float dt = (now - last_time) / 1000.0;
    if(dt <= 0 || dt > 1.0) dt = 0.01;
    
    int64_t error = setpoint - current_position;
    
    if(abs(error) <= deadband) {
      Serial.println("🎯 Within deadband → STOP");
      return 0;
    }
    
    // P Term
    float P = Kp * error;
    
    // I Term with Anti-Windup
    error_integral += error * dt;
    error_integral = constrain(error_integral, -integral_limit, integral_limit);
    float I = Ki * error_integral;
    
    // D Term with Limiting
    float error_derivative = (error - last_error) / dt;
    float D = Kd * error_derivative;
    D = constrain(D, -d_limit, d_limit);
    
    // Total PID Output
    float pid_output = P + I + D;
    int pwm = (int)abs(pid_output);
    
    // Deceleration Zones
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
      
      if(historyCount >= 2) {
        stopSpeed -= 15;
      }
      
      pwm = max(pwm, stopSpeed);
    }
    
    pwm = constrain(pwm, min_output, max_output);
    
    last_error = error;
    last_time = now;
    
    return pwm;
  }
  
  void resetPID() {
    error_integral = 0;
    last_error = 0;
    last_time = 0;
    first_call = true;
    Serial.println("✅ PID reset");
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

float getAngleDegrees() {
  int64_t pos = encoder.getCount();
  int64_t positionMod = pos % TOTAL_PULSES_PER_REV;
  if(positionMod < 0) positionMod += TOTAL_PULSES_PER_REV;
  return (positionMod * 360.0) / (float)TOTAL_PULSES_PER_REV;
}

float getRevolutions() {
  return (float)encoder.getCount() / (float)TOTAL_PULSES_PER_REV;
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

void calibrateMotor() {
  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║           🔧 MOTOR CALIBRATION 🔧             ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  
  int64_t backoffDistance = TOTAL_PULSES_PER_REV / 5; // 0.2 vòng
  
  // ===== BƯỚC 1: Tìm LIMIT 12 (trái) =====
  Serial.println("\n[1] Moving CW to find LEFT limit (Button 12)...");
  setMotorDirection(DIR_CLOCKWISE);
  setMotorSpeed(SPEED_2);
  
  unsigned long timeout = millis();
  while(digitalRead(LIMIT_SWITCH_12) == HIGH) {
    readEncoderSpeed();
    delay(10);
    
    if(millis() - timeout > 30000) {
      motorStop();
      Serial.println("❌ Timeout - Button 12 not found!");
      return;
    }
  }
  motorStop();
  Serial.println("✅ Found LEFT limit (Button 12)!");
  delay(300);
  
  // Lùi ra khỏi nút
  Serial.println("[2] Backing off from LEFT limit...");
  int64_t startPos = encoder.getCount();
  
  setMotorDirection(DIR_COUNTER_CLOCKWISE);
  setMotorSpeed(SPEED_2);
  
  timeout = millis();
  while(digitalRead(LIMIT_SWITCH_12) == LOW) {
    readEncoderSpeed();
    delay(10);
    if(millis() - timeout > 5000) break;
  }
  
  delay(200);
  int64_t currentPos = encoder.getCount();
  int64_t targetPos = currentPos - (backoffDistance / 2);
  
  while(encoder.getCount() > targetPos) {
    readEncoderSpeed();
    delay(10);
  }
  motorStop();
  
  int64_t leftLimitPos = encoder.getCount();
  Serial.printf("✅ LEFT limit position: %ld\n", (long)leftLimitPos);
  delay(500);
  
  // ===== BƯỚC 2: Tìm LIMIT 14 (phải) =====
  Serial.println("\n[3] Moving CW to find RIGHT limit (Button 14)...");
  setMotorDirection(DIR_CLOCKWISE);
  setMotorSpeed(SPEED_2);
  
  timeout = millis();
  while(digitalRead(LIMIT_SWITCH_14) == HIGH) {
    readEncoderSpeed();
    delay(10);
    
    if(millis() - timeout > 30000) {
      motorStop();
      Serial.println("❌ Timeout - Button 14 not found!");
      return;
    }
  }
  motorStop();
  Serial.println("✅ Found RIGHT limit (Button 14)!");
  delay(300);
  
  // Lùi ra khỏi nút
  Serial.println("[4] Backing off from RIGHT limit...");
  startPos = encoder.getCount();
  
  setMotorDirection(DIR_COUNTER_CLOCKWISE);
  setMotorSpeed(SPEED_2);
  
  timeout = millis();
  while(digitalRead(LIMIT_SWITCH_14) == LOW) {
    readEncoderSpeed();
    delay(10);
    if(millis() - timeout > 5000) break;
  }
  
  delay(200);
  currentPos = encoder.getCount();
  targetPos = currentPos - (backoffDistance / 2);
  
  while(encoder.getCount() > targetPos) {
    readEncoderSpeed();
    delay(10);
  }
  motorStop();
  
  int64_t rightLimitPos = encoder.getCount();
  Serial.printf("✅ RIGHT limit position: %ld\n", (long)rightLimitPos);
  
  // ===== KẾT QUẢ =====
  int64_t travelDistance = rightLimitPos - leftLimitPos;
  
  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║          ✅ CALIBRATION COMPLETED! ✅          ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  Serial.printf("\nLEFT limit:   %ld pulses\n", (long)leftLimitPos);
  Serial.printf("RIGHT limit:  %ld pulses\n", (long)rightLimitPos);
  Serial.printf("Travel range: %ld pulses (%.2f revs)\n", 
    (long)travelDistance,
    (float)travelDistance / TOTAL_PULSES_PER_REV);
  
  // Reset encoder về giữa
  Serial.println("\n[5] Moving to CENTER position...");
  int64_t centerPos = leftLimitPos + (travelDistance / 2);
  encoder.clearCount();
  
  Serial.printf("CENTER set to position 0 (physical: %ld)\n", (long)centerPos);
  
  printDetailedStatus();
}

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
    Serial.println("╚═══════════════════╝");
    return;
  }
  
  Direction dir = (targetPulses > startPos) ? DIR_CLOCKWISE : DIR_COUNTER_CLOCKWISE;
  
  if(pid.historyCount >= 2 && totalDistance > 500) {
    int64_t predictedOvershoot = pid.getPredictedOvershoot(totalDistance);
    Serial.printf("║ 🎯 Predicted overshoot: %ld pulses\n", (long)predictedOvershoot);
  }
  
  int adaptiveCrawl = pid.getAdaptiveCrawlSpeed(totalDistance);
  Serial.printf("║ Adaptive crawl: %d PWM\n", adaptiveCrawl);
  Serial.printf("║ Direction: %s | Warm: %s\n",
    dir == DIR_CLOCKWISE ? "CW" : "CCW",
    pid.motorIsWarm ? "YES" : "NO");
  Serial.println("╚═══════════════════╝");
  
  setMotorDirection(dir);
  
  unsigned long startTime = millis();
  unsigned long lastProgressTime = millis();
  int64_t lastProgressPos = startPos;
  
  while(millis() - startTime < 30000) {
    readEncoderSpeed();
    
    int64_t currentPos = encoder.getCount();
    
    int pwm = pid.compute(targetPulses, currentPos, totalDistance, dir);
    
    if(pwm == 0) {
      break;
    }
    
    setMotorSpeed(pwm);
    
    // Stuck detection
    if(millis() - lastProgressTime > 2000) {
      int64_t moved = abs(currentPos - lastProgressPos);
      if(moved < 10) {
        Serial.println("⚠️ STUCK detected!");
        break;
      }
      lastProgressTime = millis();
      lastProgressPos = currentPos;
    }
    
    // Target reached
    if(dir == DIR_CLOCKWISE && currentPos >= targetPulses) break;
    if(dir == DIR_COUNTER_CLOCKWISE && currentPos <= targetPulses) break;
    
    delay(5);
  }
  
  // Gentle Brake
  motorStop();
  delay(50);
  
  Direction brakeDir = (dir == DIR_CLOCKWISE) ? DIR_COUNTER_CLOCKWISE : DIR_CLOCKWISE;
  setMotorDirection(brakeDir);
  setMotorSpeed(160);
  delay(30);
  motorStop();
  
  delay(800);
  
  // Result & Learning
  int64_t finalPos = encoder.getCount();
  int64_t finalError = finalPos - targetPulses;
  
  pid.addMeasurement(finalError, totalDistance, dir);
  
  Serial.println("\n╔═══ RESULT ═══╗");
  Serial.printf("║ Target:  %ld\n", (long)targetPulses);
  Serial.printf("║ Actual:  %ld\n", (long)finalPos);
  Serial.printf("║ Error:   %ld pulses (%.2f°)\n",
    (long)finalError, (finalError * 360.0) / TOTAL_PULSES_PER_REV);
  
  if(abs(finalError) <= 5) {
    Serial.println("║ ✅ EXCELLENT!");
  } else if(abs(finalError) <= 15) {
    Serial.println("║ ✅ GOOD!");
  } else if(abs(finalError) <= 50) {
    Serial.println("║ ⚠️  ACCEPTABLE");
  } else {
    Serial.println("║ ❌ POOR - Learning...");
  }
  Serial.println("╚═══════════════╝");
  
  pid.printStatus();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(LIMIT_SWITCH_12, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_14, INPUT_PULLUP);
  
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RL_EN, PWM_CHANNEL);
  
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachFullQuad(ENCODER_C1, ENCODER_C2);
  encoder.setFilter(1023);
  encoder.clearCount();
  
  motorStop();
  
  Serial.println("\n╔════════════════════════════════╗");
  Serial.println("║  ESP32-S3 PID Motor Control    ║");
  Serial.println("╚════════════════════════════════╝");
  Serial.println("\n🎯 PID Commands:");
  Serial.println("  M90      - Move by 90° (relative)");
  Serial.println("  A90      - Move to 90° (absolute)");
  Serial.println("  R1.5     - Move 1.5 revolutions");
  Serial.println("  P1000    - Move to position 1000 pulses");
  
  Serial.println("\n🎮 Manual Control:");
  Serial.println("  CW1      - Clockwise Speed 1 (160 PWM)");
  Serial.println("  CW2      - Clockwise Speed 2 (190 PWM)");
  Serial.println("  CW3      - Clockwise Speed 3 (220 PWM)");
  Serial.println("  CW4      - Clockwise Speed 4 (255 PWM)");
  Serial.println("  CCW1     - Counter-Clockwise Speed 1");
  Serial.println("  CCW2     - Counter-Clockwise Speed 2");
  Serial.println("  CCW3     - Counter-Clockwise Speed 3");
  Serial.println("  CCW4     - Counter-Clockwise Speed 4");
  Serial.println("  S        - STOP motor");
  
  Serial.println("\n⚙️ System Commands:");
  Serial.println("  CAL      - Calibrate motor (find limits)");
  Serial.println("  TUNE <Kp> <Ki> <Kd> - Tune PID");
  Serial.println("  INFO     - Show PID + position status");
  Serial.println("  RESET    - Reset encoder & PID");
  Serial.println("\n>>> Ready! <<<\n");
}

void loop() {
  readEncoderSpeed();
  
  if(Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    // ===== MANUAL CONTROL =====
    if(cmd == "CW1") {
      setSpeedLevel(DIR_CLOCKWISE, SPEED_1);
      Serial.println("✓ CW Speed 1 (160 PWM)");
    }
    else if(cmd == "CW2") {
      setSpeedLevel(DIR_CLOCKWISE, SPEED_2);
      Serial.println("✓ CW Speed 2 (190 PWM)");
    }
    else if(cmd == "CW3") {
      setSpeedLevel(DIR_CLOCKWISE, SPEED_3);
      Serial.println("✓ CW Speed 3 (220 PWM)");
    }
    else if(cmd == "CW4") {
      setSpeedLevel(DIR_CLOCKWISE, SPEED_4);
      Serial.println("✓ CW Speed 4 (255 PWM)");
    }
    else if(cmd == "CCW1") {
      setSpeedLevel(DIR_COUNTER_CLOCKWISE, SPEED_1);
      Serial.println("✓ CCW Speed 1 (160 PWM)");
    }
    else if(cmd == "CCW2") {
      setSpeedLevel(DIR_COUNTER_CLOCKWISE, SPEED_2);
      Serial.println("✓ CCW Speed 2 (190 PWM)");
    }
    else if(cmd == "CCW3") {
      setSpeedLevel(DIR_COUNTER_CLOCKWISE, SPEED_3);
      Serial.println("✓ CCW Speed 3 (220 PWM)");
    }
    else if(cmd == "CCW4") {
      setSpeedLevel(DIR_COUNTER_CLOCKWISE, SPEED_4);
      Serial.println("✓ CCW Speed 4 (255 PWM)");
    }
    else if(cmd == "S" || cmd == "STOP") {
      motorStop();
      Serial.println("🛑 Motor STOPPED");
    }
    
    // ===== CALIBRATION =====
    else if(cmd == "CAL") {
      calibrateMotor();
    }
    
    // ===== PID COMMANDS =====
    else if(cmd.startsWith("M")) {
      float deg = cmd.substring(1).toFloat();
      int64_t pulses = (int64_t)((deg / 360.0) * TOTAL_PULSES_PER_REV);
      int64_t targetPos = encoder.getCount() + pulses;
      moveToPositionPID(targetPos);
    }
    else if(cmd.startsWith("A")) {
      float deg = cmd.substring(1).toFloat();
      int64_t targetPos = (int64_t)((deg / 360.0) * TOTAL_PULSES_PER_REV);
      moveToPositionPID(targetPos);
    }
    else if(cmd.startsWith("R")) {
      float revs = cmd.substring(1).toFloat();
      int64_t pulses = (int64_t)(revs * TOTAL_PULSES_PER_REV);
      int64_t targetPos = encoder.getCount() + pulses;
      moveToPositionPID(targetPos);
    }
    else if(cmd.startsWith("P")) {
      int64_t pos = cmd.substring(1).toInt();
      moveToPositionPID(pos);
    }
    else if(cmd.startsWith("TUNE")) {
      int firstSpace = cmd.indexOf(' ');
      int secondSpace = cmd.indexOf(' ', firstSpace + 1);
      if(secondSpace > 0) {
        float kp = cmd.substring(firstSpace + 1, secondSpace).toFloat();
        float ki = cmd.substring(secondSpace + 1).toFloat();
        
        int thirdSpace = cmd.indexOf(' ', secondSpace + 1);
        float kd = pid.Kd;
        if(thirdSpace > 0) {
          kd = cmd.substring(thirdSpace + 1).toFloat();
        }
        
        pid.tune(kp, ki, kd);
      }
    }
    else if(cmd == "INFO") {
      pid.printStatus();
      printDetailedStatus();
    }
    else if(cmd == "RESET") {
      encoder.clearCount();
      pid.resetPID();
      pid.moveCount = 0;
      pid.historyCount = 0;
      pid.overshootPerRev = 0;
      pid.crawlSpeedBase = 0;
      pid.motorIsWarm = false;
      Serial.println("✅ System reset!");
    }
    else {
      Serial.println("❌ Unknown command!");
    }
  }
}
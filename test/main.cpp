#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "FuncMotor.h"
#include "WifiPortal.h"

// ==================== LED RGB ====================
#define RGB_PIN 48
#define NUM_PIXELS 1
Adafruit_NeoPixel pixels(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastBlink = 0;
bool blinkState = false;

// ==================== ĐỊNH NGHĨA BIẾN KỊCH BẢN ====================
bool programRunning = false;
ScenarioStep scenario[MAX_STEPS];
int scenarioStepCount = 0;
int currentScenarioStep = 0;
unsigned long stepStartTime = 0;

// ==================== CHUYỂN ĐỔI GÓC ====================
#define PULSES_PER_REV  TOTAL_PULSES_PER_REV
#define DEGREES_PER_PULSE  (360.0 / PULSES_PER_REV)

float getCurrentAngle() {
  return encoder.getCount() * DEGREES_PER_PULSE;
}

void zeroEncoder() {
  encoder.clearCount();
  Serial.println("Encoder reset to 0");
}

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

void ledProgramRunning() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastBlink >= 200) {
    lastBlink = currentMillis;
    blinkState = !blinkState;
    if (blinkState) {
      pixels.setPixelColor(0, pixels.Color(128, 0, 128));
    } else {
      pixels.setPixelColor(0, pixels.Color(30, 0, 30));
    }
    pixels.show();
  }
}

void ledUpdate() {
  if (programRunning) {
    ledProgramRunning();
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

// ==================== XỬ LÝ KỊCH BẢN ====================
void runScenario() {
  if (!programRunning || scenarioStepCount == 0) return;
  
  yield();
  
  ScenarioStep &step = scenario[currentScenarioStep];
  float currentAngle = getCurrentAngle();
  
  bool stepComplete = false;
  
  if (step.type == 0) {  // ANGLE
    float normalizedCurrent = fmod(currentAngle, 360.0);
    if (normalizedCurrent < 0) normalizedCurrent += 360.0;
    
    float normalizedTarget = fmod(step.target, 360.0);
    if (normalizedTarget < 0) normalizedTarget += 360.0;
    
    float error = normalizedTarget - normalizedCurrent;
    
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;
    
    if (abs(error) < 1.0) {
      stepComplete = true;
    } else {
      Direction autoDir = (error > 0) ? DIR_CLOCKWISE : DIR_COUNTER_CLOCKWISE;
      setSpeedLevel(autoDir, (SpeedLevel)step.speed);
    }
    
  } else if (step.type == 1) {  // RELATIVE
    int64_t currentPos = encoder.getCount();
    float angleMoved = abs(currentPos - step.startPosition) * DEGREES_PER_PULSE;
    
    if (angleMoved >= abs(step.target)) {
      stepComplete = true;
    } else {
      setSpeedLevel((Direction)step.direction, (SpeedLevel)step.speed);
    }
    
  } else if (step.type == 2) {  // TIME
    unsigned long elapsed = millis() - stepStartTime;
    if (elapsed >= step.duration) {
      stepComplete = true;
    } else {
      setSpeedLevel((Direction)step.direction, (SpeedLevel)step.speed);
    }
  }
  
  if (stepComplete) {
    motorStop();
    delay(200);
    
    currentScenarioStep++;
    if (currentScenarioStep >= scenarioStepCount) {
      Serial.println("Scenario completed!");
      programRunning = false;
      currentScenarioStep = 0;
    } else {
      stepStartTime = millis();
      scenario[currentScenarioStep].startPosition = encoder.getCount();
      
      Serial.print("Step ");
      Serial.print(currentScenarioStep + 1);
      Serial.print("/");
      Serial.println(scenarioStepCount);
    }
  }
  
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.print("Step ");
    Serial.print(currentScenarioStep + 1);
    Serial.print(" | Angle: ");
    Serial.print(currentAngle, 2);
    Serial.print("° | Target: ");
    Serial.println(step.target, 2);
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(500);  // Give serial time to initialize
  
  Serial.println("\n=== Starting System ===");
  
  // Initialize LED
  pixels.begin();
  pixels.setBrightness(50);
  ledWifiStarting();
  
  Serial.println("Initializing motor...");
  motorSetup();
  
  // Then WiFi
  Serial.println("Starting WiFi portal...");
  WifiPortalsetup();
  
  ledWifiReady();
  
  Serial.println("\n=== System Ready ===");
  Serial.println("Angle Control Mode");
  Serial.printf("Pulses per revolution: %d\n", TOTAL_PULSES_PER_REV);
  Serial.printf("Degrees per pulse: %.3f\n", DEGREES_PER_PULSE);
}

// ==================== LOOP ====================
void loop() {
  WifiPortalloop();
  
  // Feed watchdog
  yield();
  
  // Read encoder
  readEncoderSpeed();
  if(jogMode && softLimitsEnabled) {
    int64_t pos = encoder.getCount();
    if((jogDirection == DIR_CLOCKWISE && pos >= softLimitMax) ||
        (jogDirection == DIR_COUNTER_CLOCKWISE && pos <= softLimitMin)) {
      API_stopJog();
    }
  }
  
  // Run scenario if active
  if (programRunning) {
    static unsigned long lastScenarioCheck = 0;
    if (millis() - lastScenarioCheck >= 50) {
      lastScenarioCheck = millis();
      runScenario();
    }
  }
  
  // Update LED
  static unsigned long lastLED = 0;
  if (millis() - lastLED >= 50) {
    lastLED = millis();
    ledUpdate();
  }
  
  // Process serial commands
  processSerialCommands();
  
  vTaskDelay(1);
}
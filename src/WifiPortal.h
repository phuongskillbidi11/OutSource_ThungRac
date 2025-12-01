#ifndef WIFIPORTAL_H
#define WIFIPORTAL_H

#include <Arduino.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "MotorController.h"

const char *ssid = "captive";
const char *password = NULL;

#define MAX_CLIENTS 4
#define WIFI_CHANNEL 6

const IPAddress localIP(4, 3, 2, 1);
const IPAddress gatewayIP(4, 3, 2, 1);
const IPAddress subnetMask(255, 255, 255, 0);
const String localIPURL = "http://4.3.2.1";

// ===== COMMAND QUEUE SYSTEM =====
struct MotorCommandQueue {
  struct Command {
    int motorId;
    String cmd;
  };
  
  Command commands[20];
  int head = 0;
  int tail = 0;
  int count = 0;
  
  bool push(int motorId, String cmd) {
    if(count >= 20) return false;
    commands[tail].motorId = motorId;
    commands[tail].cmd = cmd;
    tail = (tail + 1) % 20;
    count++;
    return true;
  }
  
  Command pop() {
    Command cmd = commands[head];
    head = (head + 1) % 20;
    count--;
    return cmd;
  }
  
  bool isEmpty() { return count == 0; }
  bool isFull() { return count >= 20; }
  
  void clear() {
    head = tail = count = 0;
  }
};

MotorCommandQueue commandQueue;

// ===== WEBSOCKET =====
AsyncWebSocket ws("/ws");
void broadcastStatus() {
  // KIỂM TRA MOTORS ĐÃ SẴN SÀNG CHƯA
  if(!motorsReady || motorCount == 0) {
    Serial.println("⚠️ Motors not ready, skipping broadcast");
    return;
  }
  
  DynamicJsonDocument doc(2048);
  JsonArray motorsArray = doc.createNestedArray("motors");
  
  for (int i = 0; i < motorCount; i++) {
    // KIỂM TRA TỪNG MOTOR TRƯỚC KHI TRUY CẬP
    if(!motors[i].initialized) {
      Serial.printf("⚠️ Motor %d not initialized, skipping\n", i);
      continue;
    }
    
    Motor &m = motors[i];
    JsonObject motor = motorsArray.createNestedObject();
    
    motor["id"] = m.id;
    motor["name"] = m.name;
    
    // TRUY CẬP ENCODER AN TOÀN
    int64_t position = 0;
    float angle = 0.0;
    
    try {
      position = m.encoder.getCount();
      angle = motorGetAngle(i);
    } catch(...) {
      Serial.printf("⚠️ Error reading Motor %d encoder\n", i);
      position = 0;
      angle = 0.0;
    }
    
    motor["position"] = (long)position;
    motor["angle"] = angle;
    motor["rpm"] = m.motorRPM;
    motor["direction"] = (int)m.currentDir;
    motor["speed"] = (int)m.currentSpeed;
    motor["jogMode"] = m.jogMode;
    
    JsonObject pid = motor.createNestedObject("pid");
    pid["moves"] = m.pid.moveCount;
    pid["overshoot"] = m.pid.overshootPerRev;
    pid["crawlBase"] = m.pid.crawlSpeedBase;
    pid["kp"] = m.pid.Kp;
    pid["ki"] = m.pid.Ki;
    pid["kd"] = m.pid.Kd;
    
    JsonObject limits = motor.createNestedObject("limits");
    limits["enabled"] = m.softLimitsEnabled;
    limits["min"] = (long)m.softLimitMin;
    limits["max"] = (long)m.softLimitMax;
  }
  
  doc["queue"] = commandQueue.count;
  
  String response;
  serializeJson(doc, response);
  ws.textAll(response);
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if(type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected\n", client->id());
    broadcastStatus();
  } 
  else if(type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
  else if(type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      // data[len] = 0;
      // String message = (char*)data;
      String message = String((const char*)data, len); 
      
      // JsonDocument doc;
      // DynamicJsonDocument doc(4096);
      StaticJsonDocument<256> doc; 
      DeserializationError error = deserializeJson(doc, message);
      
      if(!error) {
        int motorId = doc["motor_id"] | 0;
        String cmd = doc["command"].as<String>();
        
        if(motorId >= motorCount) {
          client->text("{\"status\":\"error\",\"message\":\"Invalid motor ID\"}");
          return;
        }
        
        // Immediate commands (don't queue)
        if(cmd == "STOP") {
          motorStop(motorId);
          commandQueue.clear();
          client->text("{\"status\":\"stopped\"}");
          return;
        }
        
        // Jog commands
        if(cmd.startsWith("JOG_CW:")) {
          int speed = cmd.substring(7).toInt();
          motorStartJog(motorId, DIR_CLOCKWISE, (SpeedLevel)speed);
          client->text("{\"status\":\"jog_started\"}");
          return;
        }
        if(cmd.startsWith("JOG_CCW:")) {
          int speed = cmd.substring(8).toInt();
          motorStartJog(motorId, DIR_COUNTER_CLOCKWISE, (SpeedLevel)speed);
          client->text("{\"status\":\"jog_started\"}");
          return;
        }
        if(cmd == "JOG_STOP") {
          motorStopJog(motorId);
          client->text("{\"status\":\"jog_stopped\"}");
          return;
        }
        
        // Soft limits
        if(cmd.startsWith("LIMITS:")) {
          int colonPos = cmd.indexOf(':', 7);
          if(colonPos > 0) {
            int64_t min = cmd.substring(7, colonPos).toInt();
            int64_t max = cmd.substring(colonPos + 1).toInt();
            motors[motorId].softLimitMin = min;
            motors[motorId].softLimitMax = max;
            motors[motorId].softLimitsEnabled = true;
            client->text("{\"status\":\"limits_set\"}");
          }
          return;
        }
        if(cmd == "LIMITS_OFF") {
          motors[motorId].softLimitsEnabled = false;
          client->text("{\"status\":\"limits_disabled\"}");
          return;
        }
        
        // PID tuning
        if(cmd.startsWith("TUNE:")) {
          int firstColon = cmd.indexOf(':', 5);
          int secondColon = cmd.indexOf(':', firstColon + 1);
          if(secondColon > 0) {
            float kp = cmd.substring(5, firstColon).toFloat();
            float ki = cmd.substring(firstColon + 1, secondColon).toFloat();
            float kd = cmd.substring(secondColon + 1).toFloat();
            motors[motorId].pid.tune(kp, ki, kd);
            client->text("{\"status\":\"pid_tuned\"}");
          }
          return;
        }
        if(cmd == "PID_RESET") {
          motors[motorId].pid.reset();
          motors[motorId].pid.resetLearning();
          client->text("{\"status\":\"pid_reset\"}");
          return;
        }
        
        // Queue move commands
        if(commandQueue.push(motorId, cmd)) {
          client->text("{\"status\":\"queued\",\"queue\":" + String(commandQueue.count) + "}");
          Serial.printf("📥 Queued M%d: %s (queue: %d)\n", motorId, cmd.c_str(), commandQueue.count);
        } else {
          client->text("{\"status\":\"error\",\"message\":\"Queue full\"}");
          Serial.println("❌ Queue full!");
        }
      }
    }
  }
}

const char index_html[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>Multi-Motor Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <style>
    * { 
      margin: 0; 
      padding: 0; 
      box-sizing: border-box;
      -webkit-tap-highlight-color: transparent;
    }
    body { 
      font-family: Arial, sans-serif;
      background: #f5f5f5;
      padding: 10px;
    }
    .container { 
      max-width: 1200px; 
      margin: 0 auto; 
    }
    .header {
      background: #333;
      color: white;
      padding: 15px;
      text-align: center;
      border-radius: 8px 8px 0 0;
      margin-bottom: 15px;
    }
    .header h2 { 
      font-size: 20px;
      margin-bottom: 5px;
    }
    .status-line {
      font-size: 13px;
      margin-top: 5px;
    }
    .dot {
      display: inline-block;
      width: 8px;
      height: 8px;
      border-radius: 50%;
      margin-right: 5px;
    }
    .dot.connected { background: #4CAF50; }
    .dot.disconnected { background: #f44336; }
    
    .motor-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
      gap: 15px;
    }
    
    .motor-card {
      background: white;
      border-radius: 8px;
      padding: 15px;
      box-shadow: 0 2px 4px rgba(0,0,0,0.1);
    }
    
    .motor-header {
      font-size: 18px;
      font-weight: bold;
      margin-bottom: 15px;
      padding-bottom: 10px;
      border-bottom: 2px solid #2196F3;
      color: #333;
    }
    
    .info-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 8px;
      margin-bottom: 15px;
    }
    .info-box {
      background: #f9f9f9;
      padding: 8px;
      border-radius: 5px;
      border: 1px solid #e0e0e0;
    }
    .info-label {
      font-size: 10px;
      color: #666;
      margin-bottom: 2px;
    }
    .info-value {
      font-size: 16px;
      font-weight: bold;
      color: #333;
    }
    
    .control-group {
      margin-bottom: 10px;
    }
    
    input[type="number"] {
      width: 100%;
      padding: 10px;
      font-size: 14px;
      border: 1px solid #ccc;
      border-radius: 5px;
      margin-bottom: 5px;
    }
    
    .btn {
      padding: 10px 15px;
      font-size: 14px;
      border: none;
      border-radius: 5px;
      cursor: pointer;
      width: 100%;
      margin-bottom: 5px;
      transition: background 0.2s;
      font-weight: bold;
    }
    .btn-primary { background: #2196F3; color: white; }
    .btn-primary:active { background: #1976D2; }
    .btn-success { background: #4CAF50; color: white; }
    .btn-success:active { background: #388E3C; }
    .btn-danger { background: #f44336; color: white; }
    .btn-danger:active { background: #d32f2f; }
    .btn-secondary { background: #9E9E9E; color: white; }
    .btn-secondary:active { background: #757575; }
    
    .quick-grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 5px;
      margin-bottom: 10px;
    }
    .quick-btn {
      padding: 8px;
      font-size: 12px;
      border: none;
      border-radius: 5px;
      background: #2196F3;
      color: white;
      cursor: pointer;
      font-weight: bold;
    }
    .quick-btn:active { background: #1976D2; }
    
    .jog-controls {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 8px;
      margin-bottom: 10px;
    }
    .jog-btn {
      padding: 30px 15px;
      font-size: 16px;
      border: none;
      border-radius: 8px;
      cursor: pointer;
      font-weight: bold;
      transition: all 0.2s;
    }
    .jog-cw {
      background: #FF9800;
      color: white;
    }
    .jog-cw:active { 
      background: #F57C00;
      transform: scale(0.95);
    }
    .jog-ccw {
      background: #9C27B0;
      color: white;
    }
    .jog-ccw:active { 
      background: #7B1FA2;
      transform: scale(0.95);
    }
    
    .pid-info {
      background: #e3f2fd;
      padding: 8px;
      border-radius: 5px;
      border-left: 3px solid #2196F3;
      margin-top: 10px;
      font-size: 11px;
    }
    .pid-param {
      display: flex;
      justify-content: space-between;
      margin: 2px 0;
    }
    
    .toast {
      position: fixed;
      bottom: 20px;
      left: 50%;
      transform: translateX(-50%);
      background: #323232;
      color: white;
      padding: 12px 24px;
      border-radius: 5px;
      font-size: 14px;
      opacity: 0;
      transition: opacity 0.3s;
      z-index: 1000;
    }
    .toast.show {
      opacity: 1;
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h2>🎛️ Multi-Motor Control System</h2>
      <div class="status-line">
        <span class="dot" id="wsStatus"></span>
        <span id="connectionStatus">Connecting...</span>
      </div>
    </div>
    
    <div class="motor-grid" id="motorGrid">
      <!-- Motors will be generated here -->
    </div>
  </div>
  
  <div class="toast" id="toast"></div>
  
  <script>
    let ws;
    let reconnectTimer;
    let motors = [];
    
    function connectWebSocket() {
      ws = new WebSocket('ws://4.3.2.1/ws');
      
      ws.onopen = () => {
        console.log('WebSocket connected');
        document.getElementById('connectionStatus').textContent = 'Connected';
        document.getElementById('wsStatus').className = 'dot connected';
        clearTimeout(reconnectTimer);
      };
      
      ws.onclose = () => {
        console.log('WebSocket disconnected');
        document.getElementById('connectionStatus').textContent = 'Disconnected';
        document.getElementById('wsStatus').className = 'dot disconnected';
        reconnectTimer = setTimeout(connectWebSocket, 2000);
      };
      
      ws.onerror = (error) => {
        console.error('WebSocket error:', error);
      };
      
      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if(data.motors) {
            motors = data.motors;
            if(document.getElementById('motorGrid').children.length === 0) {
              renderMotors();
            }
            updateDisplay(data);
          }
        } catch(e) {
          console.error('Parse error:', e);
        }
      };
    }
    
    function renderMotors() {
      const grid = document.getElementById('motorGrid');
      grid.innerHTML = motors.map((m, idx) => `
        <div class="motor-card">
          <div class="motor-header">⚙️ ${m.name}</div>
          
          <div class="info-grid">
            <div class="info-box">
              <div class="info-label">Position</div>
              <div class="info-value" id="pos-${m.id}">0</div>
            </div>
            <div class="info-box">
              <div class="info-label">Angle</div>
              <div class="info-value" id="angle-${m.id}">0°</div>
            </div>
            <div class="info-box">
              <div class="info-label">RPM</div>
              <div class="info-value" id="rpm-${m.id}">0</div>
            </div>
            <div class="info-box">
              <div class="info-label">PWM</div>
              <div class="info-value" id="pwm-${m.id}">0</div>
            </div>
          </div>
          
          <div class="pid-info">
            <div class="pid-param">
              <span>Moves:</span>
              <span id="pid-moves-${m.id}">0</span>
            </div>
            <div class="pid-param">
              <span>Overshoot:</span>
              <span id="pid-overshoot-${m.id}">0</span>
            </div>
            <div class="pid-param">
              <span>Kp|Ki|Kd:</span>
              <span id="pid-gains-${m.id}">-|-|-</span>
            </div>
          </div>
          
          <div class="control-group">
            <div class="quick-grid">
              <button class="quick-btn" onclick="quick(${m.id}, -90)">-90°</button>
              <button class="quick-btn" onclick="quick(${m.id}, -45)">-45°</button>
              <button class="quick-btn" onclick="quick(${m.id}, 45)">+45°</button>
              <button class="quick-btn" onclick="quick(${m.id}, 90)">+90°</button>
            </div>
          </div>
          
          <div class="control-group">
            <input type="number" id="deg-${m.id}" placeholder="Degrees">
            <button class="btn btn-primary" onclick="moveBy(${m.id})">Move By</button>
          </div>
          
          <div class="control-group">
            <input type="number" id="angle-${m.id}" placeholder="Target Angle">
            <button class="btn btn-success" onclick="moveTo(${m.id})">Go To</button>
          </div>
          
          <div class="jog-controls">
            <button class="jog-btn jog-cw" 
              ontouchstart="jogStart(${m.id}, 'CW')" ontouchend="jogStop(${m.id})"
              onmousedown="jogStart(${m.id}, 'CW')" onmouseup="jogStop(${m.id})">
              CW ▶
            </button>
            <button class="jog-btn jog-ccw" 
              ontouchstart="jogStart(${m.id}, 'CCW')" ontouchend="jogStop(${m.id})"
              onmousedown="jogStart(${m.id}, 'CCW')" onmouseup="jogStop(${m.id})">
              ◀ CCW
            </button>
          </div>
          
          <button class="btn btn-danger" onclick="stop(${m.id})">🛑 STOP</button>
          <button class="btn btn-secondary" onclick="reset(${m.id})">🔄 Reset</button>
        </div>
      `).join('');
    }
    
    function updateDisplay(data) {
      data.motors.forEach(m => {
        const pos = document.getElementById('pos-' + m.id);
        const angle = document.getElementById('angle-' + m.id);
        const rpm = document.getElementById('rpm-' + m.id);
        const pwm = document.getElementById('pwm-' + m.id);
        
        if(pos) pos.textContent = m.position;
        if(angle) angle.textContent = m.angle.toFixed(1) + '°';
        if(rpm) rpm.textContent = m.rpm.toFixed(1);
        if(pwm) pwm.textContent = m.speed;
        
        if(m.pid) {
          const moves = document.getElementById('pid-moves-' + m.id);
          const overshoot = document.getElementById('pid-overshoot-' + m.id);
          const gains = document.getElementById('pid-gains-' + m.id);
          
          if(moves) moves.textContent = m.pid.moves;
          if(overshoot) overshoot.textContent = m.pid.overshoot.toFixed(1);
          if(gains) gains.textContent = 
            m.pid.kp.toFixed(1) + '|' + 
            m.pid.ki.toFixed(3) + '|' + 
            m.pid.kd.toFixed(1);
        }
      });
    }
    
    function sendCommand(motorId, cmd) {
      if(ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({motor_id: motorId, command: cmd}));
      } else {
        showToast('Not connected!');
      }
    }
    
    function showToast(message) {
      const toast = document.getElementById('toast');
      toast.textContent = message;
      toast.classList.add('show');
      setTimeout(() => toast.classList.remove('show'), 2000);
    }
    
    function moveBy(id) {
      const input = document.getElementById('deg-' + id);
      const deg = input.value;
      if(deg) {
        sendCommand(id, 'M' + deg);
        input.value = '';
        showToast('M' + id + ': Moving ' + deg + '°');
      }
    }
    
    function moveTo(id) {
      const input = document.getElementById('angle-' + id);
      const angle = input.value;
      if(angle) {
        sendCommand(id, 'A' + angle);
        input.value = '';
        showToast('M' + id + ': Going to ' + angle + '°');
      }
    }
    
    function quick(id, deg) {
      sendCommand(id, 'M' + deg);
      showToast('M' + id + ': ' + deg + '°');
    }
    
    function stop(id) {
      sendCommand(id, 'STOP');
      showToast('M' + id + ' STOPPED');
    }
    
    function reset(id) {
      sendCommand(id, 'RESET');
      showToast('M' + id + ' Reset');
    }
    
    function jogStart(id, dir) {
      sendCommand(id, 'JOG_' + dir + ':2');
    }
    
    function jogStop(id) {
      sendCommand(id, 'JOG_STOP');
    }
    
    connectWebSocket();
  </script>
</body>
</html>
)=====";

DNSServer dnsServer;
AsyncWebServer server(80);

void setUpDNSServer(DNSServer &dnsServer, const IPAddress &localIP) {
  #define DNS_INTERVAL 30
  dnsServer.setTTL(3600);
  dnsServer.start(53, "*", localIP);
}

void startSoftAccessPoint(const char *ssid, const char *password, const IPAddress &localIP, const IPAddress &gatewayIP) {
  WiFi.mode(WIFI_MODE_AP);
  const IPAddress subnetMask(255, 255, 255, 0);
  WiFi.softAPConfig(localIP, gatewayIP, subnetMask);
  WiFi.softAP(ssid, password, WIFI_CHANNEL, 0, MAX_CLIENTS);
}

void setUpWebserver(AsyncWebServer &server, const IPAddress &localIP) {
  // Captive portal redirects
  server.on("/connecttest.txt", [](AsyncWebServerRequest *request) { request->redirect("http://logout.net"); });
  server.on("/wpad.dat", [](AsyncWebServerRequest *request) { request->send(404); });
  server.on("/generate_204", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });
  server.on("/redirect", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });
  server.on("/hotspot-detect.html", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });
  server.on("/canonical.html", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });
  server.on("/success.txt", [](AsyncWebServerRequest *request) { request->send(200); });
  server.on("/ncsi.txt", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });
  server.on("/favicon.ico", [](AsyncWebServerRequest *request) { request->send(404); });

  // Main page
  server.on("/", HTTP_ANY, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(200, "text/html", index_html);
    response->addHeader("Cache-Control", "public,max-age=31536000");
    request->send(response);
  });

  server.onNotFound([](AsyncWebServerRequest *request) {
    request->redirect(localIPURL);
  });
  
  // WebSocket handler
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
}

// ===== COMMAND PROCESSOR =====
void processCommandQueue() {
  // ⚠️ CRITICAL - THÊM DÒNG NÀY
  if (!motorsReady) return;
  
  static unsigned long lastProcess = 0;
  
  if(!commandQueue.isEmpty() && millis() - lastProcess > 100) {
    lastProcess = millis();
    
    MotorCommandQueue::Command cmd = commandQueue.pop();
    int motorId = cmd.motorId;
    String command = cmd.cmd;
    command.toUpperCase();
    
    if(motorId >= motorCount) return;
    
    Serial.printf("⚙️ Processing M%d: %s (remaining: %d)\n", motorId, command.c_str(), commandQueue.count);
    
    Motor &m = motors[motorId];
    
    if(command.startsWith("M")) {
      float deg = command.substring(1).toFloat();
      int64_t pulses = (int64_t)((deg / 360.0) * m.pulsesPerRev);
      int64_t targetPos = m.encoder.getCount() + pulses;
      motorMovePID(motorId, targetPos);
    }
    else if(command.startsWith("A")) {
      float targetAngle = command.substring(1).toFloat();
      
      while(targetAngle < 0) targetAngle += 360;
      while(targetAngle >= 360) targetAngle -= 360;
      
      float currentAngle = motorGetAngle(motorId);
      while(currentAngle < 0) currentAngle += 360;
      while(currentAngle >= 360) currentAngle -= 360;
      
      float angleDiff = targetAngle - currentAngle;
      if(angleDiff > 180) angleDiff -= 360;
      if(angleDiff < -180) angleDiff += 360;
      
      int64_t pulses = (int64_t)((angleDiff / 360.0) * m.pulsesPerRev);
      int64_t targetPos = m.encoder.getCount() + pulses;
      
      motorMovePID(motorId, targetPos);
    }
    else if(command.startsWith("R")) {
      float revs = command.substring(1).toFloat();
      int64_t pulses = (int64_t)(revs * m.pulsesPerRev);
      int64_t targetPos = m.encoder.getCount() + pulses;
      motorMovePID(motorId, targetPos);
    }
    else if(command == "RESET") {
      motorZeroPosition(motorId);
    }
    
    // Broadcast queue status
    // JsonDocument doc;
   StaticJsonDocument<128> doc;
    doc["queue"] = commandQueue.count;
    String json;
    serializeJson(doc, json);
    ws.textAll(json);
  }
}

// ===== STATUS BROADCASTER =====
void broadcastStatusPeriodic() {
  static unsigned long lastBroadcast = 0;
  if(millis() - lastBroadcast > 200) {
    lastBroadcast = millis();
    broadcastStatus();
  }
}

void WifiPortalsetup() {
  Serial.println("\n\nMulti-Motor Control Portal with WebSocket");
  Serial.printf("%s-%d\n\r", ESP.getChipModel(), ESP.getChipRevision());

  startSoftAccessPoint(ssid, password, localIP, gatewayIP);
  setUpDNSServer(dnsServer, localIP);
  setUpWebserver(server, localIP);
  server.begin();
  
  Serial.println("✓ Portal active: " + String(ssid));
  Serial.println("✓ URL: " + localIPURL);
  Serial.println("✓ WebSocket: ws://4.3.2.1/ws");
  Serial.printf("Startup Time: %lu ms\n\n", millis());
}

void WifiPortalloop() {
  dnsServer.processNextRequest();
  ws.cleanupClients();
  processCommandQueue();
  broadcastStatusPeriodic();
}

#endif
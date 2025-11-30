#include <Arduino.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ArduinoJson.h>

const char *ssid = "captive";
const char *password = NULL;

#define MAX_CLIENTS 4
#define WIFI_CHANNEL 6

const IPAddress localIP(4, 3, 2, 1);
const IPAddress gatewayIP(4, 3, 2, 1);
const IPAddress subnetMask(255, 255, 255, 0);
const String localIPURL = "http://4.3.2.1";

// ===== EXTERNAL DECLARATIONS =====
enum Direction;
enum SpeedLevel;

extern bool programRunning;
extern int currentSpeed;
extern Direction currentDirection;
extern float motorRPM;

struct CompensationSystem;
extern CompensationSystem compensation;

extern void API_moveByDegrees(float degrees);
extern void API_moveToAbsoluteDegrees(float degrees);
extern void API_moveRevolutions(float revolutions);
extern String API_getCurrentPosition();
extern String API_getStatus();
extern void API_resetPosition();
extern void API_stopMotor();
extern float getAngleDegrees();
extern void motorStop();
// extern bool motorTaskRunning;

struct ScenarioStep {
  int type;
  float target;
  int direction;
  int speed;
  int duration;
  long startPosition;
};
#define MAX_STEPS 10
extern ScenarioStep scenario[MAX_STEPS];
extern int scenarioStepCount;
extern int currentScenarioStep;
extern unsigned long stepStartTime;

// ✅ COMMAND QUEUE SYSTEM
struct MotorCommandQueue {
  String commands[10];
  int head = 0;
  int tail = 0;
  int count = 0;
  
  bool push(String cmd) {
    if(count >= 10) return false;
    commands[tail] = cmd;
    tail = (tail + 1) % 10;
    count++;
    return true;
  }
  
  String pop() {
    if(count == 0) return "";
    String cmd = commands[head];
    head = (head + 1) % 10;
    count--;
    return cmd;
  }
  
  bool isEmpty() { return count == 0; }
  bool isFull() { return count >= 10; }
  
  void clear() {
    head = tail = count = 0;
  }
};

MotorCommandQueue commandQueue;

// ✅ WEBSOCKET FOR REAL-TIME COMMUNICATION
AsyncWebSocket ws("/ws");

void broadcastStatus() {
  // Get base status
  String baseStatus = API_getStatus();
  
  // Remove closing brace from baseStatus
  baseStatus.remove(baseStatus.length() - 1);
  
  // Add compensation data
  baseStatus += ",\"compensation\":{";
  baseStatus += "\"overshoot\":" + String(compensation.overshootPerRev, 1) + ",";
  baseStatus += "\"crawlBase\":" + String(compensation.crawlSpeedBase) + ",";
  baseStatus += "\"moves\":" + String(compensation.moveCount);
  baseStatus += "},";
  
  // Add limits
  baseStatus += "\"limits\":{";
  baseStatus += "\"enabled\":" + String(softLimitsEnabled ? "true" : "false") + ",";
  baseStatus += "\"min\":" + String((long)softLimitMin) + ",";
  baseStatus += "\"max\":" + String((long)softLimitMax);
  baseStatus += "},";
  
  // Add config
  baseStatus += "\"maxSpeed\":" + String(maxSpeedLimit) + ",";
  baseStatus += "\"jogMode\":" + String(jogMode ? "true" : "false");
  
  baseStatus += "}";
  
  ws.textAll(baseStatus);
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if(type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected\n", client->id());
    // Send initial status
    client->text(API_getStatus());
  } 
  else if(type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
  else if(type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      String message = (char*)data;
      
      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, message);
      
      if(!error) {
          String cmd = doc["command"].as<String>();
  
         //  THÊM XỬ LÝ COMMANDS ĐẶC BIỆT (KHÔNG CẦN QUEUE):
        
        // Immediate commands (don't queue)
        if(cmd == "STOP" || cmd == "BRAKE") {
          if(cmd == "BRAKE") {
            API_emergencyBrake();
          } else {
            API_stopMotor();
          }
          commandQueue.clear();
          client->text("{\"status\":\"stopped\"}");
          return;
        }
        
        // Jog commands
        if(cmd.startsWith("JOG_CW:")) {
          int speed = cmd.substring(7).toInt();
          API_startJog(DIR_CLOCKWISE, speed);
          client->text("{\"status\":\"jog_started\"}");
          return;
        }
        if(cmd.startsWith("JOG_CCW:")) {
          int speed = cmd.substring(8).toInt();
          API_startJog(DIR_COUNTER_CLOCKWISE, speed);
          client->text("{\"status\":\"jog_started\"}");
          return;
        }
        if(cmd == "JOG_STOP") {
          API_stopJog();
          client->text("{\"status\":\"jog_stopped\"}");
          return;
        }
        
        // Speed setting
        if(cmd.startsWith("SPEED:")) {
          int speed = cmd.substring(6).toInt();
          API_setMaxSpeed(speed);
          client->text("{\"status\":\"speed_set\",\"speed\":" + String(maxSpeedLimit) + "}");
          return;
        }
        
        // Soft limits
        if(cmd.startsWith("LIMITS:")) {
          int colonPos = cmd.indexOf(':', 7);
          if(colonPos > 0) {
            int64_t min = cmd.substring(7, colonPos).toInt();
            int64_t max = cmd.substring(colonPos + 1).toInt();
            API_setSoftLimits(min, max);
            client->text("{\"status\":\"limits_set\"}");
          }
          return;
        }
        if(cmd == "LIMITS_OFF") {
          API_disableSoftLimits();
          client->text("{\"status\":\"limits_disabled\"}");
          return;
        }
        
        // ✅ INSTANT RESPONSE - Add to queue
        if(commandQueue.push(cmd)) {
          client->text("{\"status\":\"queued\",\"queue\":" + String(commandQueue.count) + "}");
          Serial.printf("📥 Queued: %s (queue: %d)\n", cmd.c_str(), commandQueue.count);
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
  <title>Motor Control</title>
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
      max-width: 600px; 
      margin: 0 auto; 
      background: white;
      border: 1px solid #ddd;
    }
    .header {
      background: #333;
      color: white;
      padding: 15px;
      text-align: center;
      border-bottom: 3px solid #666;
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
      background: #0f0;
      margin-right: 5px;
    }
    .dot.off { background: #f00; }
    
    .content { padding: 15px; }
    
    .info-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-bottom: 15px;
      border: 1px solid #ddd;
      padding: 10px;
      background: #fafafa;
    }
    .info-item {
      text-align: center;
      padding: 8px;
    }
    .info-label {
      font-size: 11px;
      color: #666;
      text-transform: uppercase;
    }
    .info-value {
      font-size: 22px;
      font-weight: bold;
      margin-top: 3px;
    }
    .info-item.moving .info-value { color: #f80; }
    
    .section {
      margin-bottom: 15px;
      padding: 15px;
      border: 1px solid #ddd;
      background: #fff;
    }
    .section-title {
      font-size: 14px;
      font-weight: bold;
      margin-bottom: 10px;
      color: #333;
    }
    
    input[type="number"] {
      width: 100%;
      padding: 12px;
      border: 2px solid #ddd;
      font-size: 18px;
      text-align: center;
    }
    input[type="number"]:focus {
      outline: none;
      border-color: #666;
    }
    
    input[type="range"] {
      width: 100%;
      height: 40px;
    }
    
    button {
      width: 100%;
      padding: 15px;
      border: none;
      font-size: 16px;
      font-weight: bold;
      cursor: pointer;
      margin-top: 8px;
      text-transform: uppercase;
      touch-action: manipulation;
    }
    button:active { 
      opacity: 0.7;
    }
    
    .btn-red { background: #d00; color: white; }
    .btn-gray { background: #666; color: white; }
    .btn-green { background: #0a0; color: white; }
    .btn-blue { background: #06c; color: white; }
    
    .grid-3 {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
    }
    .grid-3 button {
      margin-top: 0;
      padding: 12px 8px;
      font-size: 14px;
    }
    
    .grid-2 {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
    }
    .grid-2 button {
      margin-top: 0;
    }
    
    .queue-bar {
      background: #ffc;
      border: 1px solid #cc0;
      padding: 8px;
      text-align: center;
      font-size: 13px;
      margin-bottom: 10px;
      display: none;
    }
    .queue-bar.show { display: block; }
    
    .toast {
      position: fixed;
      bottom: 20px;
      left: 50%;
      transform: translateX(-50%);
      background: #333;
      color: white;
      padding: 12px 24px;
      border-radius: 4px;
      font-size: 14px;
      opacity: 0;
      transition: opacity 0.3s;
      pointer-events: none;
      z-index: 1000;
    }
    .toast.show { opacity: 0.95; }
    
    .speed-label {
      text-align: center;
      font-size: 18px;
      font-weight: bold;
      margin-top: 8px;
    }
    
    .comp-info {
      font-size: 13px;
      line-height: 1.6;
    }
    .comp-info div {
      padding: 4px 0;
      border-bottom: 1px dotted #ddd;
    }
    .comp-info div:last-child { border: none; }
    
    @media (max-width: 480px) {
      .header h2 { font-size: 18px; }
      .info-value { font-size: 20px; }
      button { font-size: 14px; padding: 12px; }
      .grid-3 button { font-size: 13px; padding: 10px 5px; }
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h2>MOTOR CONTROL</h2>
      <div class="status-line">
        <span class="dot" id="wsDot"></span>
        <span id="wsStatus">Connecting...</span>
      </div>
    </div>
    
    <div class="content">
      <div class="queue-bar" id="queueInfo">
        Queue: <span id="queueCount">0</span> commands
      </div>
      
      <div class="info-grid">
        <div class="info-item">
          <div class="info-label">Angle</div>
          <div class="info-value"><span id="angle">0</span>&deg;</div>
        </div>
        <div class="info-item">
          <div class="info-label">Position</div>
          <div class="info-value" id="position">0</div>
        </div>
        <div class="info-item">
          <div class="info-label">RPM</div>
          <div class="info-value" id="rpm">0.0</div>
        </div>
        <div class="info-item" id="statusBox">
          <div class="info-label">Status</div>
          <div class="info-value" id="direction">STOP</div>
        </div>
      </div>
      
      <div class="section">
        <button class="btn-red" onclick="stop()">EMERGENCY STOP</button>
        <button class="btn-gray" onclick="reset()">RESET POSITION</button>
      </div>
      
      <div class="section">
        <div class="section-title">Move by Degrees</div>
        <input type="number" id="moveDeg" placeholder="Enter degrees">
        <button class="btn-green" onclick="moveBy()">MOVE RELATIVE</button>
      </div>
      
      <div class="section">
        <div class="section-title">Move to Angle</div>
        <input type="number" id="moveAngle" placeholder="0-360" min="0" max="360">
        <button class="btn-green" onclick="moveTo()">GO TO ABSOLUTE</button>
      </div>
      
      <div class="section">
        <div class="section-title">Quick Actions</div>
        <div class="grid-3">
          <button class="btn-blue" onclick="quick(45)">+45&deg;</button>
          <button class="btn-blue" onclick="quick(90)">+90&deg;</button>
          <button class="btn-blue" onclick="quick(180)">+180&deg;</button>
          <button class="btn-blue" onclick="quick(-45)">-45&deg;</button>
          <button class="btn-blue" onclick="quick(-90)">-90&deg;</button>
          <button class="btn-blue" onclick="quick(-180)">-180&deg;</button>
        </div>
      </div>
      
      <div class="section">
        <div class="section-title">Max Speed Control</div>
        <input type="range" id="speedSlider" min="160" max="255" value="250" 
               oninput="updateSpeedLabel(this.value)" 
               onchange="setMaxSpeed(this.value)">
        <div class="speed-label">
          <span id="speedLabel">250</span> PWM
        </div>
      </div>

      <div class="section">
        <div class="section-title">Jog Control (Hold Button)</div>
        <div class="grid-2">
          <button class="btn-blue" 
                  onmousedown="jogStart('CW')" 
                  onmouseup="jogStop()"
                  ontouchstart="jogStart('CW')" 
                  ontouchend="jogStop()">
            CW JOG &gt;
          </button>
          <button class="btn-blue" 
                  onmousedown="jogStart('CCW')" 
                  onmouseup="jogStop()"
                  ontouchstart="jogStart('CCW')" 
                  ontouchend="jogStop()">
            &lt; CCW JOG
          </button>
        </div>
      </div>

      <div class="section">
        <div class="section-title">Compensation Info</div>
        <div class="comp-info">
          <div>Overshoot/rev: <strong id="overshoot">0</strong> pulses</div>
          <div>Crawl base: <strong id="crawlBase">0</strong> PWM</div>
          <div>Total moves: <strong id="totalMoves">0</strong></div>
        </div>
      </div>
    </div>
  </div>
  
  <div class="toast" id="toast"></div>

  <script>
    let ws;
    let reconnectInterval;
    
    function showToast(msg) {
      const toast = document.getElementById('toast');
      toast.textContent = msg;
      toast.classList.add('show');
      setTimeout(() => toast.classList.remove('show'), 2000);
    }
    
    function connectWebSocket() {
      ws = new WebSocket('ws://' + window.location.hostname + '/ws');
      
      ws.onopen = () => {
        document.getElementById('wsStatus').textContent = 'Connected';
        document.getElementById('wsDot').classList.remove('off');
        clearInterval(reconnectInterval);
      };
      
      ws.onclose = () => {
        document.getElementById('wsStatus').textContent = 'Disconnected';
        document.getElementById('wsDot').classList.add('off');
        reconnectInterval = setInterval(connectWebSocket, 2000);
      };
      
      ws.onerror = (error) => {
        console.error('WebSocket error:', error);
      };
      
      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          
          if(data.angle !== undefined) {
            document.getElementById('angle').textContent = parseFloat(data.angle).toFixed(1);
            document.getElementById('position').textContent = data.position;
            document.getElementById('rpm').textContent = parseFloat(data.rpm).toFixed(1);
            document.getElementById('direction').textContent = data.direction;
            
            const box = document.getElementById('statusBox');
            if(data.busy || data.direction !== 'STOP') {
              box.classList.add('moving');
            } else {
              box.classList.remove('moving');
            }
          }
          
          if(data.compensation) {
            document.getElementById('overshoot').textContent = 
              parseFloat(data.compensation.overshoot).toFixed(1);
            document.getElementById('crawlBase').textContent = 
              data.compensation.crawlBase;
            document.getElementById('totalMoves').textContent = 
              data.compensation.moves;
          }
          
          if(data.queue !== undefined) {
            const queueInfo = document.getElementById('queueInfo');
            document.getElementById('queueCount').textContent = data.queue;
            if(data.queue > 0) {
              queueInfo.classList.add('show');
            } else {
              queueInfo.classList.remove('show');
            }
          }
          
          if(data.status === 'queued') {
            showToast('Command queued');
          } else if(data.status === 'error') {
            showToast('ERROR: ' + data.message);
          }
        } catch(e) {
          console.error('Parse error:', e);
        }
      };
    }
    
    function sendCommand(cmd) {
      if(ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({command: cmd}));
      } else {
        showToast('Not connected');
      }
    }
    
    function moveBy() {
      const deg = document.getElementById('moveDeg').value;
      if(deg) {
        sendCommand('M' + deg);
        document.getElementById('moveDeg').value = '';
      }
    }
    
    function moveTo() {
      const angle = document.getElementById('moveAngle').value;
      if(angle) {
        sendCommand('A' + angle);
        document.getElementById('moveAngle').value = '';
      }
    }
    
    function quick(deg) {
      sendCommand('M' + deg);
    }
    
    function stop() {
      sendCommand('STOP');
      showToast('EMERGENCY STOP');
    }
    
    function reset() {
      if(confirm('Reset position to 0?')) {
        sendCommand('RESET');
        showToast('Position reset');
      }
    }
    
    function updateSpeedLabel(value) {
      document.getElementById('speedLabel').textContent = value;
    }

    function setMaxSpeed(value) {
      sendCommand('SPEED:' + value);
      showToast('Speed set to ' + value);
    }

    function jogStart(dir) {
      sendCommand('JOG_' + dir + ':200');
    }

    function jogStop() {
      sendCommand('JOG_STOP');
    }
    
    document.getElementById('moveDeg').addEventListener('keypress', (e) => {
      if(e.key === 'Enter') moveBy();
    });
    document.getElementById('moveAngle').addEventListener('keypress', (e) => {
      if(e.key === 'Enter') moveTo();
    });
    
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

  // ✅ LEGACY HTTP API - Still supported but not recommended
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", API_getStatus());
  });

  server.on("/api/stop", HTTP_POST, [](AsyncWebServerRequest *request) {
    commandQueue.push("STOP");
    request->send(200, "application/json", "{\"success\":true}");
  });

  server.on("/api/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    commandQueue.push("RESET");
    request->send(200, "application/json", "{\"success\":true}");
  });
  server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send(200, "application/json", API_getAllConfig());
  });

  server.on("/api/compensation", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", API_getCompensationStatus());
  });

  server.on("/api/speed", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      deserializeJson(doc, data);
      int speed = doc["speed"];
      API_setMaxSpeed(speed);
      request->send(200, "application/json", "{\"success\":true,\"speed\":" + String(maxSpeedLimit) + "}");
    }
  );

  server.on("/api/jog", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      deserializeJson(doc, data);
      String action = doc["action"].as<String>();
      
      if(action == "start") {
        String dir = doc["direction"].as<String>();
        int speed = doc["speed"] | 200;
        Direction direction = (dir == "CW") ? DIR_CLOCKWISE : DIR_COUNTER_CLOCKWISE;
        API_startJog(direction, speed);
        request->send(200, "application/json", "{\"success\":true}");
      } else if(action == "stop") {
        API_stopJog();
        request->send(200, "application/json", "{\"success\":true}");
      }
    }
  );

  server.on("/api/limits", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<128> doc;
      deserializeJson(doc, data);
      
      if(doc["enable"] == false) {
        API_disableSoftLimits();
        request->send(200, "application/json", "{\"success\":true}");
      } else {
        int64_t min = doc["min"];
        int64_t max = doc["max"];
        API_setSoftLimits(min, max);
        request->send(200, "application/json", "{\"success\":true}");
      }
    }
  );

  server.on("/api/brake", HTTP_POST, [](AsyncWebServerRequest *request) {
    API_emergencyBrake();
    commandQueue.clear();
    request->send(200, "application/json", "{\"success\":true}");
  });

  server.onNotFound([](AsyncWebServerRequest *request) {
    request->redirect(localIPURL);
  });
  
  // ✅ WebSocket handler
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
}

// ✅ COMMAND PROCESSOR - Runs in main loop
void processCommandQueue() {
  static unsigned long lastProcess = 0;
  
  // Only process if motor is idle and queue has commands
  if(!motorTaskRunning && !commandQueue.isEmpty() && millis() - lastProcess > 100) {
    lastProcess = millis();
    
    String cmd = commandQueue.pop();
    cmd.toUpperCase();
    
    Serial.printf("⚙️ Processing: %s (remaining: %d)\n", cmd.c_str(), commandQueue.count);
    
    if(cmd.startsWith("M")) {
      float deg = cmd.substring(1).toFloat();
      API_moveByDegrees(deg);
    }
    else if(cmd.startsWith("A")) {
      float angle = cmd.substring(1).toFloat();
      API_moveToAbsoluteDegrees(angle);
    }
    else if(cmd.startsWith("R")) {
      float revs = cmd.substring(1).toFloat();
      API_moveRevolutions(revs);
    }
    else if(cmd == "STOP") {
      API_stopMotor();
      commandQueue.clear();
    }
    else if(cmd == "RESET") {
      API_resetPosition();
    }
    
    // Broadcast queue status
    StaticJsonDocument<128> doc;
    doc["queue"] = commandQueue.count;
    String json;
    serializeJson(doc, json);
    ws.textAll(json);
  }
}

// ✅ STATUS BROADCASTER - Runs every 200ms
void broadcastStatusPeriodic() {
  static unsigned long lastBroadcast = 0;
  if(millis() - lastBroadcast > 200) {
    lastBroadcast = millis();
    broadcastStatus();
  }
}

void WifiPortalsetup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\n\nMotor Control Portal with WebSocket");
  Serial.printf("%s-%d\n\r", ESP.getChipModel(), ESP.getChipRevision());

  startSoftAccessPoint(ssid, password, localIP, gatewayIP);
  setUpDNSServer(dnsServer, localIP);
  setUpWebserver(server, localIP);
  server.begin();
  
  Serial.println("✓ Portal active: " + String(ssid));
  Serial.println("✓ URL: " + localIPURL);
  Serial.println("\n🌐 WebSocket ready at: ws://4.3.2.1/ws");
  Serial.println("✓ Command queue system active");
  Serial.printf("Startup Time: %lu ms\n\n", millis());
}

void WifiPortalloop() {
  dnsServer.processNextRequest();
  ws.cleanupClients();  // Clean disconnected clients
  processCommandQueue();  // Process queued commands
  broadcastStatusPeriodic();  // Broadcast status updates
  delay(10);
}
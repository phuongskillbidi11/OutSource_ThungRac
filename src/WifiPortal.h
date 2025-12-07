#ifndef WIFIPORTAL_H
#define WIFIPORTAL_H

#include <Arduino.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "MotorController.h"
#include "FlowConfig.h"
#include "FlowController.h" 
#include "SystemMode.h" 
const char *ssid = "captive";
const char *password = NULL;

#define MAX_CLIENTS 4
#define WIFI_CHANNEL 6

const IPAddress localIP(4, 3, 2, 1);
const IPAddress gatewayIP(4, 3, 2, 1);
const IPAddress subnetMask(255, 255, 255, 0);
const String localIPURL = "http://4.3.2.1";

// ===== COMMAND QUEUE =====
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
  void clear() { head = tail = count = 0; }
};

MotorCommandQueue commandQueue;

// ===== WEBSOCKET =====
AsyncWebSocket ws("/ws");

void broadcastStatus() {
  if(!motorsReady || motorCount == 0) return;
  
  DynamicJsonDocument doc(2048);
  JsonArray motorsArray = doc.createNestedArray("motors");
  
  for (int i = 0; i < motorCount; i++) {
    if(!motors[i].initialized) continue;
    
    Motor &m = motors[i];
    JsonObject motor = motorsArray.createNestedObject();
    
    motor["id"] = m.id;
    motor["name"] = m.name;
    motor["position"] = (long)m.encoder.getCount();
    motor["angle"] = motorGetAngle(i);
    motor["rpm"] = m.motorRPM;
    motor["direction"] = (int)m.currentDir;
    motor["speed"] = (int)m.currentSpeed;
    motor["ppr"] = m.pulsesPerRev;
    
    JsonObject pid = motor.createNestedObject("pid");
    pid["kp"] = m.pid.Kp;
    pid["ki"] = m.pid.Ki;
    pid["kd"] = m.pid.Kd;
    pid["moves"] = m.pid.moveCount;
  }
  //Mode info
    doc["system_mode"] = (int)sysState.currentMode;
    doc["mode_name"] = getCurrentModeString();
    doc["status_msg"] = sysState.statusMessage;
    doc["flows_active"] = countActiveFlows();
    doc["queue"] = commandQueue.count;
  
  String response;
  serializeJson(doc, response);
  ws.textAll(response);
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if(type == WS_EVT_CONNECT) {
    Serial.printf("✓ WS Client #%u connected\n", client->id());
    if(!motorsReady) {
      Serial.println("⚠ Motors not ready yet");
    }
    broadcastStatus();
  } 
  else if(type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket #%u disconnected\n", client->id());
  }
  else if(type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      String message = String((const char*)data, len);
      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, message);
      
      if(!error) {
        int motorId = doc["motor_id"] | 0;
        String cmd = doc["command"].as<String>();
        
        if(motorId >= motorCount) {
          client->text("{\"status\":\"error\"}");
          return;
        }
        // ===== CHECK MODE RESTRICTIONS =====
        if(sysState.currentMode == MODE_AUTO && 
           (cmd.startsWith("JOG_") || cmd.startsWith("M") || 
            cmd.startsWith("A") || cmd.startsWith("R"))) {
          client->text("{\"status\":\"blocked\",\"message\":\"Manual control disabled in AUTO mode\"}");
          return;
        }
        
        if(cmd == "STOP") {
          motorStop(motorId);
          commandQueue.clear();
          client->text("{\"status\":\"stopped\"}");
          return;
        }
        
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
        
        if(cmd == "RESET") {
          motorZeroPosition(motorId);
          client->text("{\"status\":\"reset\"}");
          return;
        }
        // =====  MODE COMMANDS ===== 
        if(cmd == "SWITCH_MANUAL") {
          if(switchToManualMode()) {
            client->text("{\"status\":\"mode_manual\"}");
            broadcastStatus();
          } else {
            String err = "{\"status\":\"error\",\"message\":\"" + sysState.statusMessage + "\"}";
            client->text(err);
          }
          return;
        }
        
        if(cmd == "SWITCH_AUTO") {
          if(switchToAutoMode()) {
            client->text("{\"status\":\"mode_auto\"}");
            broadcastStatus();
          } else {
            String err = "{\"status\":\"error\",\"message\":\"" + sysState.statusMessage + "\"}";
            client->text(err);
          }
          return;
        }
        
        if(cmd == "EMERGENCY_STOP") {
          switchToIdleMode();
          client->text("{\"status\":\"emergency_stopped\"}");
          broadcastStatus();
          return;
        }
        // ===== END MODE COMMANDS =====
        
        // Queue move commands
        if(commandQueue.push(motorId, cmd)) {
          client->text("{\"status\":\"queued\"}");
        } else {
          client->text("{\"status\":\"queue_full\"}");
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
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:Arial,sans-serif;background:#f5f5f5;padding:8px}
.container{max-width:1400px;margin:0 auto}
.header{background:#333;color:#fff;padding:10px;text-align:center;margin-bottom:8px;border-radius:4px;font-size:18px}
.status{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:5px}
.on{background:#4CAF50}.off{background:#f44336}
.tabs{display:flex;gap:4px;margin-bottom:8px}
.tab{flex:1;padding:8px;background:#666;color:#fff;border:none;cursor:pointer;border-radius:4px;font-weight:bold;font-size:14px}
.tab.active{background:#2196F3}
.tab.disabled{background:#444;color:#999;cursor:not-allowed;opacity:0.6}
.tab-content{display:none}
.tab-content.active{display:block}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:8px}
.card{background:#fff;padding:10px;border-radius:4px;box-shadow:0 1px 3px rgba(0,0,0,0.1)}
.title{font-weight:bold;margin-bottom:8px;padding-bottom:5px;border-bottom:2px solid #2196F3;font-size:15px}
.row{display:flex;justify-content:space-between;padding:4px 0;font-size:13px}
.label{color:#666}
.value{font-weight:bold}
input,textarea{width:100%;padding:6px;border:1px solid #ccc;border-radius:3px;font-size:13px;margin-bottom:4px}
textarea{font-family:monospace;height:250px;resize:vertical}
.btn{width:100%;padding:8px;border:none;border-radius:3px;cursor:pointer;font-weight:bold;margin-bottom:4px;font-size:13px}
.blue{background:#2196F3;color:#fff}
.green{background:#4CAF50;color:#fff}
.red{background:#f44336;color:#fff}
.orange{background:#FF9800;color:#fff}
.purple{background:#9C27B0;color:#fff}
.gray{background:#999;color:#fff}
.btns{display:grid;grid-template-columns:repeat(4,1fr);gap:4px;margin-bottom:4px}
.btn-sm{padding:6px;background:#2196F3;color:#fff;border:none;border-radius:3px;cursor:pointer;font-size:12px}
.jog{display:grid;grid-template-columns:1fr 1fr;gap:4px;margin-bottom:4px}
.jog-btn{padding:20px;border:none;border-radius:4px;cursor:pointer;font-weight:bold;font-size:14px}
.cfg-item{margin-bottom:8px}
.cfg-item label{display:block;font-size:12px;color:#666;margin-bottom:3px}
.cfg-item input{margin-bottom:0}
.inline{display:grid;grid-template-columns:1fr 1fr 1fr;gap:4px}
/* MODE INDICATOR */
.mode-bar{
  background:#333;
  color:#fff;
  padding:8px 12px;
  margin-bottom:8px;
  border-radius:4px;
  display:flex;
  justify-content:space-between;
  align-items:center;
  font-size:13px;
}
.mode-badge{
  padding:4px 12px;
  border-radius:20px;
  font-weight:bold;
  font-size:12px;
}
.mode-idle{background:#999;color:#fff}
.mode-manual{background:#FF9800;color:#fff}
.mode-auto{background:#4CAF50;color:#fff}
.mode-msg{font-size:11px;color:#ccc;margin-top:2px}
.btn-mode{padding:6px 12px;background:#2196F3;color:#fff;border:none;border-radius:3px;cursor:pointer;font-size:11px;font-weight:bold}
.btn-emergency{background:#f44336}
.flow-item{
  border:1px solid #ddd;
  padding:10px;
  margin-bottom:8px;
  border-radius:4px;
  background:#f9f9f9;
}
.flow-header{
  display:flex;
  justify-content:space-between;
  margin-bottom:8px;
  padding-bottom:6px;
  border-bottom:1px solid #ddd;
}
.flow-name{
  font-weight:bold;
  font-size:14px;
}
.flow-badge{
  padding:3px 8px;
  border-radius:12px;
  font-size:11px;
  font-weight:bold;
}
.badge-active{background:#4CAF50;color:#fff}
.badge-inactive{background:#999;color:#fff}
.badge-ir{background:#2196F3;color:#fff}
.badge-touch{background:#9C27B0;color:#fff}
.flow-info{
  display:grid;
  grid-template-columns:1fr 1fr;
  gap:6px;
  font-size:12px;
}
.flow-info-item{
  display:flex;
  justify-content:space-between;
  padding:4px 0;
}
.flow-info-label{color:#666}
.flow-info-value{font-weight:bold}
.progress-bar{
  background:#ddd;
  height:20px;
  border-radius:10px;
  overflow:hidden;
  margin:6px 0;
  position:relative;
}
.progress-fill{
  background:#4CAF50;
  height:100%;
  transition:width 0.3s;
}
.progress-text{
  position:absolute;
  left:50%;
  top:50%;
  transform:translate(-50%,-50%);
  font-size:11px;
  font-weight:bold;
  color:#333;
}
.state-badge{
  display:inline-block;
  padding:4px 10px;
  border-radius:12px;
  font-size:11px;
  font-weight:bold;
  margin-top:6px;
}
.state-idle{background:#999;color:#fff}
.state-wait{background:#FF9800;color:#fff}
.state-recovery{background:#f44336;color:#fff}
.state-touch{background:#9C27B0;color:#fff}
.state-relay{background:#4CAF50;color:#fff}
</style>
</head>
<body>
<div class="container">
<div class="header">
⚙️ Motor Control
<div style="font-size:12px;margin-top:5px"><span class="status off" id="st"></span><span id="txt">Connecting...</span></div>
</div>
<div class="mode-bar">
  <div>
    <div>
      <strong>System Mode:</strong> 
      <span class="mode-badge mode-idle" id="modeBadge">IDLE</span>
    </div>
    <div class="mode-msg" id="modeMsg">System ready</div>
  </div>
  <div>
    <button class="btn-mode btn-emergency" onclick="emergencyStop()">
      🛑 EMERGENCY STOP
    </button>
  </div>
</div>

<div class="tabs">
<button class="tab" onclick="showTab(0)">Control</button>
<button class="tab active" onclick="showTab(1)">Config</button>
<button class="tab" onclick="showTab(2)">Flow</button>
</div>

<div id="tab0" class="tab-content">
<div class="grid" id="motors"></div>
</div>
<div id="tab1" class="tab-content active">
<div class="grid">
<div class="card">
<div class="title">Motor Settings</div>
<div id="settings"></div>
</div>
<div class="card">
<div class="title">JSON Editor</div>
<textarea id="json"></textarea>
<button class="btn blue" onclick="save()">💾 Save to ESP32</button>
<button class="btn gray" onclick="load()">🔄 Reload</button>
</div>
</div>
</div>
<div id="tab2" class="tab-content">  <!-- THÊM TAB MỚI -->
<div class="grid">
<div class="card">
<div class="title">🔄 Flow Status</div>
<div id="flowstatus"></div>
</div>
<div class="card">
<div class="title">Flow Settings</div>
<div id="flowsettings"></div>
</div>
<div class="card">
<div class="title">Flow JSON Editor</div>
<textarea id="flowjson"></textarea>
<button class="btn blue" onclick="saveFlow()">💾 Save Flow Config</button>
<button class="btn gray" onclick="loadFlow()">🔄 Reload Flow</button>
</div>
</div>
</div>

</div>

<script>
let ws,motors=[],cfg={},flowCfg={}; 
let selectedMotor=0;
let sysMode = 0;         
let lastActiveTab = 1;  
function updateTabStates() {
  let tabs = document.querySelectorAll('.tab');
  
  // Tab 0 = Control, Tab 1 = Config, Tab 2 = Flow
  
  if(sysMode === 1) {
    // MANUAL mode: Control enabled, Flow disabled
    tabs[0].classList.remove('disabled');
    tabs[1].classList.remove('disabled');
    tabs[2].classList.add('disabled');
  } else if(sysMode === 2) {
    // AUTO mode: Control disabled, Flow enabled
    tabs[0].classList.add('disabled');
    tabs[1].classList.remove('disabled');
    tabs[2].classList.remove('disabled');
  } else {
    // IDLE mode: All enabled
    tabs[0].classList.remove('disabled');
    tabs[1].classList.remove('disabled');
    tabs[2].classList.remove('disabled');
  }
}

function emergencyStop() {
  if(confirm('EMERGENCY STOP: Stop all motors and flows?')) {
    ws.send(JSON.stringify({motor_id:0,command:'EMERGENCY_STOP'}));
  }
}
function showTab(n){
  // Hide all tabs
  for(let i=0;i<3;i++){
    document.getElementById('tab'+i).classList.remove('active');
    document.querySelectorAll('.tab')[i].classList.remove('active');
  }
  
  // Show selected tab
  document.getElementById('tab'+n).classList.add('active');
  document.querySelectorAll('.tab')[n].classList.add('active');
  
  // ===== THÊM MODE HANDLING ===== (SAU show selected tab)
  // Tab 0 = Control (Manual), Tab 1 = Config (No change), Tab 2 = Flow (Auto)
  if(n === 0) {
    // Switching to Control tab - request MANUAL mode
    if(sysMode !== 1) { // 1 = MODE_MANUAL
      if(confirm('Switch to MANUAL mode? This will stop all flows.')) {
        ws.send(JSON.stringify({motor_id:0,command:'SWITCH_MANUAL'}));
      } else {
        // User cancelled, stay on current tab
        showTab(lastActiveTab);
        return;
      }
    }
  } else if(n === 2) {
    // Switching to Flow tab - request AUTO mode
    if(sysMode !== 2) { // 2 = MODE_AUTO
      if(confirm('Switch to AUTO mode? Manual control will be disabled.')) {
        ws.send(JSON.stringify({motor_id:0,command:'SWITCH_AUTO'}));
      } else {
        // User cancelled, stay on current tab
        showTab(lastActiveTab);
        return;
      }
    }
  } else if(n === 1) {
    // Config tab - can stay in any mode
  }
  
  lastActiveTab = n;
  // ===== END MODE HANDLING =====
}

function loadFlow(){
  fetch('/flow_config.json')
    .then(r=>r.text())
    .then(t=>{
      try{
        flowCfg=JSON.parse(t);
        document.getElementById('flowjson').value=JSON.stringify(flowCfg,null,2);
        renderFlowSettings();
      }catch(e){
        console.error('Parse error:', e);
        document.getElementById('flowjson').value=t;
      }
    })
    .catch(e=>console.error('Fetch error:', e));  // <-- THÊM
}

function renderFlowSettings(){
if(!flowCfg.flows)return;
updateTabStatus(); // ✅ Update tab status after loading flow config
document.getElementById('flowsettings').innerHTML=flowCfg.flows.map((f,i)=>`
<div style="border:1px solid #ddd;padding:8px;margin-bottom:8px;border-radius:4px">
<div style="font-weight:bold;margin-bottom:6px;display:flex;justify-content:space-between;align-items:center">
<span>${f.name}</span>
<button class="btn-sm ${f.enabled?'green':'gray'}" onclick="toggleFlow(${i})">${f.enabled?'ON':'OFF'}</button>
</div>
<div class="cfg-item">
<label>Motor ID</label>
<input type="number" value="${f.motor_id}" onchange="updFlow(${i},'motor_id',+this.value)">
</div>
<div class="cfg-item">
<label>Sensor Type</label>
<select onchange="updFlow(${i},'sensor_type',this.value)">
<option value="ir" ${f.sensor.type==='ir'?'selected':''}>IR Sensor (Auto detect)</option>
<option value="touch" ${f.sensor.type==='touch'?'selected':''}>Touch Sensor (Hold & Release)</option>
<option value="completion" ${f.sensor.type==='completion'?'selected':''}>Completion Trigger (Auto after Flow)</option>
</select>
</div>
<div class="cfg-item">
<label>Sensor Pin ${f.sensor.type==='completion'?'(Not used)':''}</label>
<input type="number" value="${f.pins.sensor}" onchange="updFlow(${i},'sensor',+this.value)" ${f.sensor.type==='completion'?'disabled':''}>
</div>
<div class="cfg-item">
<label>Limit Pins (CW | CCW)</label>
<div class="inline">
<input type="number" value="${f.pins.limit_cw}" onchange="updFlow(${i},'limit_cw',+this.value)" placeholder="CW">
<input type="number" value="${f.pins.limit_ccw}" onchange="updFlow(${i},'limit_ccw',+this.value)" placeholder="CCW">
<span></span>
</div>
</div>
<div class="cfg-item">
<label>Relay Pin</label>
<input type="number" value="${f.pins.relay}" onchange="updFlow(${i},'relay_pin',+this.value)">
</div>
<div class="cfg-item">
<label>Move Angle (°)</label>
<input type="number" step="0.1" value="${f.movement.angle}" onchange="updFlow(${i},'angle',+this.value)">
</div>
<div class="cfg-item">
<label>Speed (Min | Max)</label>
<div class="inline">
<input type="number" value="${f.movement.min_speed}" onchange="updFlow(${i},'min_speed',+this.value)">
<input type="number" value="${f.movement.max_speed}" onchange="updFlow(${i},'max_speed',+this.value)">
<span></span>
</div>
</div>
<div class="cfg-item">
<label>Timeout (ms)</label>
<input type="number" value="${f.movement.timeout}" onchange="updFlow(${i},'timeout',+this.value)">
</div>
<div class="cfg-item">
<label>
${f.sensor.type==='completion'?'⏳ Wait Before Return (ms)':'⏳ Sensor Clear Time (ms)'}
<small style="display:block;color:#666;font-weight:normal">
${f.sensor.type==='completion'?'Time to wait before returning to zero':'Time sensor must be clear before return'}
</small>
</label>
<input type="number" value="${f.sensor.clear_time}" onchange="updFlow(${i},'clear_time',+this.value)">
</div>
<div class="cfg-item">
<label>
${f.sensor.type==='touch'?'👆 Touch Hold Time (ms)':f.sensor.type==='completion'?'⏰ Trigger Delay (ms)':'⏱️ Hold Time (ms)'}
<small style="display:block;color:#666;font-weight:normal">
${f.sensor.type==='touch'?'Time to hold touch before trigger':f.sensor.type==='completion'?'Delay after previous flow complete':'Not used for IR sensors'}
</small>
</label>
<input type="number" value="${f.sensor.hold_time}" onchange="updFlow(${i},'hold_time',+this.value)" ${f.sensor.type==='ir'?'disabled':''}>
</div>
<div class="cfg-item">
<label>Debounce Time (ms)</label>
<input type="number" value="${f.sensor.debounce_time}" onchange="updFlow(${i},'debounce',+this.value)">
</div>
<div class="cfg-item">
<label>Relay Duration (ms)</label>
<input type="number" value="${f.relay.duration}" onchange="updFlow(${i},'relay_duration',+this.value)">
</div>
<div class="cfg-item">
<label>Relay Trigger Timeout (ms)</label>
<input type="number" value="${f.relay.trigger_timeout||10000}" onchange="updFlow(${i},'relay_trigger_timeout',+this.value)">
</div>
<div class="cfg-item">
<label>Relay Logic</label>
<div class="inline">
<label style="display:flex;align-items:center;gap:8px">
<input type="checkbox" ${f.relay.inverted?'checked':''} onchange="updFlow(${i},'relay_inverted',this.checked)">
<span>Inverted (LOW=ON)</span>
</label>
</div>
</div>
<div class="cfg-item">
<label>PID (Kp | Ki | Kd)</label>
<div class="inline">
<input type="number" step="0.1" value="${f.pid.kp}" onchange="updFlow(${i},'kp',+this.value)">
<input type="number" step="0.001" value="${f.pid.ki}" onchange="updFlow(${i},'ki',+this.value)">
<input type="number" step="0.1" value="${f.pid.kd}" onchange="updFlow(${i},'kd',+this.value)">
</div>
</div>
</div>
`).join('');
}

function toggleFlow(idx){
flowCfg.flows[idx].enabled=!flowCfg.flows[idx].enabled;
document.getElementById('flowjson').value=JSON.stringify(flowCfg,null,2);
renderFlowSettings();
}

function updFlow(idx,key,val){
const f=flowCfg.flows[idx];
if(key==='motor_id')f.motor_id=val;
else if(key==='sensor')f.pins.sensor=val;
else if(key==='limit_cw')f.pins.limit_cw=val;
else if(key==='limit_ccw')f.pins.limit_ccw=val;
else if(key==='angle')f.movement.angle=val;
else if(key==='min_speed')f.movement.min_speed=val;
else if(key==='max_speed')f.movement.max_speed=val;
else if(key==='timeout')f.movement.timeout=val;
else if(key==='clear_time')f.sensor.clear_time=val;
else if(key==='debounce')f.sensor.debounce_time=val;
else if(key==='kp')f.pid.kp=val;
else if(key==='ki')f.pid.ki=val;
else if(key==='kd')f.pid.kd=val;
else if(key==='hold_time')f.sensor.hold_time=val;
else if(key==='sensor_type')f.sensor.type=val;
else if(key==='relay_pin')f.pins.relay=val;
else if(key==='relay_duration')f.relay.duration=val;
else if(key==='relay_inverted')f.relay.inverted=val;
else if(key==='relay_trigger_timeout')f.relay.trigger_timeout=val;
document.getElementById('flowjson').value=JSON.stringify(flowCfg,null,2);
}

function saveFlow(){
const txt=document.getElementById('flowjson').value;
try{
flowCfg=JSON.parse(txt);
fetch('/save-flow-config',{method:'POST',body:txt})
.then(r=>r.text())
.then(msg=>{
alert(msg);
return fetch('/reload-flows');
})
.then(r=>r.text())
.then(msg=>{
alert(msg);
setTimeout(()=>location.reload(),500);
});
}catch(e){
alert('Invalid JSON: '+e.message);
}
}
function connectWS(){
ws=new WebSocket('ws://4.3.2.1/ws');
ws.onopen=()=>{
document.getElementById('st').className='status on';
document.getElementById('txt').textContent='Connected';
};
ws.onmessage=function(e){
  let d=JSON.parse(e.data);
  
  // ===== THÊM MODE UPDATE ===== (THÊM ĐẦU TIÊN)
  if(d.system_mode !== undefined) {
    sysMode = d.system_mode;
    
    // Update mode badge
    let badge = document.getElementById('modeBadge');
    let msg = document.getElementById('modeMsg');
    
    badge.className = 'mode-badge';
    if(sysMode === 0) {
      badge.classList.add('mode-idle');
      badge.textContent = 'IDLE';
    } else if(sysMode === 1) {
      badge.classList.add('mode-manual');
      badge.textContent = 'MANUAL';
    } else if(sysMode === 2) {
      badge.classList.add('mode-auto');
      badge.textContent = 'AUTO';
    }
    
    msg.textContent = d.status_msg || '';
    
    // Update tab states based on mode
    updateTabStates();
  }
if(d.motors){
motors=d.motors;
if(!document.getElementById('motors').innerHTML)renderMotors();
updateMotors(d);
if(d.flows){
  updateFlowStatus(d.flows);
}
}
};
}

function renderMotors(){
document.getElementById('motors').innerHTML=motors.map(m=>`
<div class="card">
<div class="title">⚙️ ${m.name}</div>
<div class="row"><span class="label">Position</span><span class="value" id="p${m.id}">0</span></div>
<div class="row"><span class="label">Angle</span><span class="value" id="a${m.id}">0°</span></div>
<div class="row"><span class="label">RPM</span><span class="value" id="r${m.id}">0</span></div>
<div class="row"><span class="label">PPR</span><span class="value">${m.ppr}</span></div>
<div class="btns">
<button class="btn-sm" onclick="q(${m.id},-90)">-90°</button>
<button class="btn-sm" onclick="q(${m.id},-45)">-45°</button>
<button class="btn-sm" onclick="q(${m.id},45)">+45°</button>
<button class="btn-sm" onclick="q(${m.id},90)">+90°</button>
</div>
<input type="number" id="d${m.id}" placeholder="Degrees">
<button class="btn blue" onclick="moveBy(${m.id})">Move By</button>
<input type="number" id="g${m.id}" placeholder="Target Angle">
<button class="btn green" onclick="moveTo(${m.id})">Go To</button>
<div class="jog">
<button class="jog-btn orange" onmousedown="jog(${m.id},'CW',1)" onmouseup="jog(${m.id},'CW',0)" ontouchstart="jog(${m.id},'CW',1)" ontouchend="jog(${m.id},'CW',0)">CW ▶</button>
<button class="jog-btn purple" onmousedown="jog(${m.id},'CCW',1)" onmouseup="jog(${m.id},'CCW',0)" ontouchstart="jog(${m.id},'CCW',1)" ontouchend="jog(${m.id},'CCW',0)">◀ CCW</button>
</div>
<button class="btn red" onclick="stop(${m.id})">🛑 STOP</button>
<button class="btn gray" onclick="reset(${m.id})">🔄 Reset</button>
</div>
`).join('');
}

function updateMotors(d){
d.motors.forEach(m=>{
const p=document.getElementById('p'+m.id);
const a=document.getElementById('a'+m.id);
const r=document.getElementById('r'+m.id);
if(p)p.textContent=m.position;
if(a)a.textContent=m.angle.toFixed(1)+'°';
if(r)r.textContent=m.rpm.toFixed(1);
});
}
function updateFlowStatus(flows){
  const statusDiv = document.getElementById('flowstatus');
  if(!statusDiv) return;
  
  if(!flows || flows.length === 0) {
    statusDiv.innerHTML = '<div style="color:#999;font-size:12px;padding:10px">No flows configured</div>';
    return;
  }
  
  let html = '';
  flows.forEach(f => {
    // State names
    const stateNames = ['IDLE', 'WAITING', 'RECOVERY', 'TOUCH HOLD', 'RELAY ACTIVE'];
    const stateName = stateNames[f.state] || 'UNKNOWN';
    const stateClasses = ['state-idle', 'state-wait', 'state-recovery', 'state-touch', 'state-relay'];
    const stateClass = stateClasses[f.state] || 'state-idle';
    
    // Type badge
    const typeBadge = f.type === 'touch' ? 
      '<span class="flow-badge badge-touch">TOUCH</span>' : 
      '<span class="flow-badge badge-ir">IR</span>';
    
    // Active badge
    const activeBadge = f.active ? 
      '<span class="flow-badge badge-active">ACTIVE</span>' : 
      '<span class="flow-badge badge-inactive">INACTIVE</span>';
    
    html += `
    <div class="flow-item">
      <div class="flow-header">
        <div>
          <span class="flow-name">${f.name}</span>
          ${typeBadge}
        </div>
        ${activeBadge}
      </div>
      
      <div class="flow-info">
        <div class="flow-info-item">
          <span class="flow-info-label">Motor:</span>
          <span class="flow-info-value">Motor ${f.motor_id}</span>
        </div>
        <div class="flow-info-item">
          <span class="flow-info-label">Sensor:</span>
          <span class="flow-info-value">${f.sensor_detected ? '🟢 DETECTED' : '⚪ CLEAR'}</span>
        </div>
        <div class="flow-info-item">
          <span class="flow-info-label">Angle:</span>
          <span class="flow-info-value">${f.angle}°</span>
        </div>
        <div class="flow-info-item">
          <span class="flow-info-label">Limits:</span>
          <span class="flow-info-value">
            ${f.limit_cw ? '🔴' : '⚪'} CW 
            ${f.limit_ccw ? '🔴' : '⚪'} CCW
          </span>
        </div>
      </div>`;
      if(f.type === 'completion' && f.trigger_pending && f.trigger_remaining > 0) {
        html += `
        <div style="text-align:center;padding:6px;background:#9C27B0;color:#fff;border-radius:4px;font-size:12px;margin-top:6px">
          ⏰ Trigger in: ${(f.trigger_remaining/1000).toFixed(1)}s
        </div>`;
      }

      // ⏳ Wait clear countdown (for Completion flow after move)
      if(f.type === 'completion' && f.wait_remaining > 0) {
        html += `
        <div style="text-align:center;padding:6px;background:#FF9800;color:#fff;border-radius:4px;font-size:12px;margin-top:6px">
          ⏳ Waiting clear: ${(f.wait_remaining/1000).toFixed(1)}s
        </div>`;
      }

      // 🔘 Relay trigger waiting (THÊM CHO TẤT CẢ FLOW)
      if(f.relay_trigger_waiting) {
        const progress = ((f.relay_trigger_elapsed / f.relay_trigger_timeout) * 100).toFixed(0);
        html += `
        <div class="progress-bar" style="background:#E3F2FD">
          <div class="progress-fill" style="width:${progress}%;background:#2196F3"></div>
          <div class="progress-text">🔘 Waiting for button: ${(f.relay_trigger_remaining/1000).toFixed(0)}s left</div>
        </div>`;
      }
    
    // Touch hold progress
    if(f.type === 'touch' && f.hold_remaining > 0) {
      const progress = ((f.hold_duration / f.hold_required) * 100).toFixed(0);
      html += `
      <div class="progress-bar">
        <div class="progress-fill" style="width:${progress}%"></div>
        <div class="progress-text">Touch Hold: ${(f.hold_remaining/1000).toFixed(1)}s left</div>
      </div>`;
    }
    
    // Wait clear countdown
    if(f.type === 'ir' && f.wait_remaining > 0) {
      html += `
      <div style="text-align:center;padding:6px;background:#FF9800;color:#fff;border-radius:4px;font-size:12px;margin-top:6px">
        ⏳ Waiting clear: ${(f.wait_remaining/1000).toFixed(1)}s
      </div>`;
    }
    
    // Relay countdown
    if(f.relay_active && f.relay_remaining > 0) {
      const progress = (((f.relay_duration - f.relay_remaining) / f.relay_duration) * 100).toFixed(0);
      html += `
      <div class="progress-bar" style="background:#FFE5E5">
        <div class="progress-fill" style="width:${progress}%;background:#4CAF50"></div>
        <div class="progress-text">🔌 Relay ON: ${(f.relay_remaining/1000).toFixed(1)}s left</div>
      </div>`;
    }
    
    // State badge
    html += `
      <div>
        <span class="state-badge ${stateClass}">${stateName}</span>
      </div>
    </div>`;
  });
  
  statusDiv.innerHTML = html;
}
function send(id,cmd){
if(ws&&ws.readyState===WebSocket.OPEN){
ws.send(JSON.stringify({motor_id:id,command:cmd}));
}
}

function moveBy(id){
// ===== THÊM MODE CHECK Ở ĐÂY =====
if(sysMode === 2) {
  alert('⚠️ Manual control blocked in AUTO mode');
  return;
}
// ===== END MODE CHECK =====

const v=document.getElementById('d'+id).value;
if(v){send(id,'M'+v);document.getElementById('d'+id).value='';}
}
function moveTo(id){
// ===== THÊM MODE CHECK Ở ĐÂY =====
if(sysMode === 2) {
  alert('⚠️ Manual control blocked in AUTO mode');
  return;
}
// ===== END MODE CHECK =====

const v=document.getElementById('g'+id).value;
if(v){send(id,'A'+v);document.getElementById('g'+id).value='';}
}
function q(id,deg){
// ===== THÊM MODE CHECK Ở ĐÂY =====
if(sysMode === 2) {
  alert('⚠️ Manual control blocked in AUTO mode');
  return;
}
// ===== END MODE CHECK =====

send(id,'M'+deg);
}
function stop(id){
  send(id,'STOP');  // ← KHÔNG check mode, stop luôn được phép
}
function reset(id){
// ===== THÊM MODE CHECK Ở ĐÂY (OPTIONAL) =====
if(sysMode === 2) {
  if(!confirm('Reset position in AUTO mode? This may affect flow automation.')) {
    return;
  }
}
// ===== END MODE CHECK =====

send(id,'RESET');
}

function jog(id,dir,start){
// ===== THÊM MODE CHECK Ở ĐÂY =====
if(start && sysMode === 2) {
  alert('⚠️ Manual control blocked in AUTO mode');
  return;
}
// ===== END MODE CHECK =====

if(start)send(id,'JOG_'+dir+':2');
else send(id,'JOG_STOP');
}
function load(){
fetch('/config.json')
.then(r=>r.text())
.then(t=>{
try{
cfg=JSON.parse(t);
document.getElementById('json').value=JSON.stringify(cfg,null,2);
renderSettings();
fetch('/reload-motors')
.then(r=>r.text())
.then(msg=>console.log(msg));
}catch(e){
document.getElementById('json').value=t;
}
});
}

function renderSettings(){
if(!cfg.motors)return;
updateTabStatus(); // ✅ Update tab status after loading config
document.getElementById('settings').innerHTML=cfg.motors.map((m,i)=>`
<div style="border:1px solid #ddd;padding:8px;margin-bottom:8px;border-radius:4px">
<div style="font-weight:bold;margin-bottom:6px">${m.name}</div>
<div class="cfg-item">
<label>Name</label>
<input type="text" value="${m.name}" onchange="upd(${i},'name',this.value)">
</div>
<div class="cfg-item">
<label>PPR (Pulses Per Rev)</label>
<input type="number" value="${m.encoder.pulses_per_rev}" onchange="upd(${i},'ppr',+this.value)">
</div>
<div class="cfg-item">
<label>PID Gains (Kp | Ki | Kd)</label>
<div class="inline">
<input type="number" step="0.1" value="${m.pid.kp}" onchange="upd(${i},'kp',+this.value)" placeholder="Kp">
<input type="number" step="0.001" value="${m.pid.ki}" onchange="upd(${i},'ki',+this.value)" placeholder="Ki">
<input type="number" step="0.1" value="${m.pid.kd}" onchange="upd(${i},'kd',+this.value)" placeholder="Kd">
</div>
</div>
<div class="cfg-item">
  <label>PID PWM (Min | Max)</label>
  <div class="inline">
    <input type="number" value="${m.pid.min_output !== undefined ? m.pid.min_output : 195}"
           onchange="upd(${i},'pid_min',+this.value)" placeholder="Min PWM">
    <input type="number" value="${m.pid.max_output !== undefined ? m.pid.max_output : 255}"
           onchange="upd(${i},'pid_max',+this.value)" placeholder="Max PWM">
    <span></span>
  </div>
</div>
<div class="cfg-item">
<label>Pins (EN | R_PWM | L_PWM)</label>
<div class="inline">
<input type="number" value="${m.pins.rl_en}" onchange="upd(${i},'en',+this.value)" placeholder="EN">
<input type="number" value="${m.pins.r_pwm}" onchange="upd(${i},'rpwm',+this.value)" placeholder="R">
<input type="number" value="${m.pins.l_pwm}" onchange="upd(${i},'lpwm',+this.value)" placeholder="L">
</div>
</div>
<div class="cfg-item">
<label>Encoder Pins (C1 | C2)</label>
<div class="inline">
<input type="number" value="${m.pins.enc_c1}" onchange="upd(${i},'c1',+this.value)" placeholder="C1">
<input type="number" value="${m.pins.enc_c2}" onchange="upd(${i},'c2',+this.value)" placeholder="C2">
<span></span>
</div>
</div>
<div class="cfg-item">
<label>Soft Limits (Min | Max)</label>
<div class="cfg-item">
<label>Speed Levels (S0 | S1 | S2 | S3 | S4)</label>
<div style="display:grid;grid-template-columns:repeat(5,1fr);gap:4px">
<input type="number" value="${m.speeds?m.speeds[0]:0}" onchange="upd(${i},'s0',+this.value)" placeholder="0">
<input type="number" value="${m.speeds?m.speeds[1]:64}" onchange="upd(${i},'s1',+this.value)" placeholder="S1">
<input type="number" value="${m.speeds?m.speeds[2]:128}" onchange="upd(${i},'s2',+this.value)" placeholder="S2">
<input type="number" value="${m.speeds?m.speeds[3]:192}" onchange="upd(${i},'s3',+this.value)" placeholder="S3">
<input type="number" value="${m.speeds?m.speeds[4]:255}" onchange="upd(${i},'s4',+this.value)" placeholder="S4">
</div>
</div>
<div class="inline">
<input type="number" value="${m.limits.soft_min}" onchange="upd(${i},'min',+this.value)" placeholder="Min">
<input type="number" value="${m.limits.soft_max}" onchange="upd(${i},'max',+this.value)" placeholder="Max">
<span></span>
</div>
</div>
</div>
`).join('');
}

function upd(idx,key,val){
const m=cfg.motors[idx];
if(key==='name')m.name=val;
else if(key==='ppr')m.encoder.pulses_per_rev=val;
else if(key==='kp')m.pid.kp=val;
else if(key==='ki')m.pid.ki=val;
else if(key==='kd')m.pid.kd=val;
else if(key==='pid_min') {
  if(!m.pid) m.pid = {kp:1,ki:0,kd:0};
  m.pid.min_output = val;
}
else if(key==='pid_max') {
  if(!m.pid) m.pid = {kp:1,ki:0,kd:0};
  m.pid.max_output = val;
}
else if(key==='en')m.pins.rl_en=val;
else if(key==='rpwm')m.pins.r_pwm=val;
else if(key==='lpwm')m.pins.l_pwm=val;
else if(key==='c1')m.pins.enc_c1=val;
else if(key==='c2')m.pins.enc_c2=val;
else if(key==='min')m.limits.soft_min=val;
else if(key==='max')m.limits.soft_max=val;
else if(key==='s0'){if(!m.speeds)m.speeds=[0,64,128,192,255];m.speeds[0]=val;}
else if(key==='s1'){if(!m.speeds)m.speeds=[0,64,128,192,255];m.speeds[1]=val;}
else if(key==='s2'){if(!m.speeds)m.speeds=[0,64,128,192,255];m.speeds[2]=val;}
else if(key==='s3'){if(!m.speeds)m.speeds=[0,64,128,192,255];m.speeds[3]=val;}
else if(key==='s4'){if(!m.speeds)m.speeds=[0,64,128,192,255];m.speeds[4]=val;}
document.getElementById('json').value=JSON.stringify(cfg,null,2);
}

function save(){
const txt=document.getElementById('json').value;
try{
cfg=JSON.parse(txt);
fetch('/save-config',{method:'POST',body:txt})
.then(r=>r.text())
.then(msg=>{
alert(msg);
return fetch('/reload-motors');
})
.then(r=>r.text())
.then(msg=>{
alert(msg);
setTimeout(()=>location.reload(),500);
});
}catch(e){
alert('Invalid JSON: '+e.message);
}
}

function updateTabStatus(){
const controlTab = document.querySelector('.tab:nth-child(1)');
const flowTab = document.querySelector('.tab:nth-child(3)');

// Check if motors config exists
const hasMotors = cfg.motors && cfg.motors.length > 0;
if(hasMotors){
controlTab.classList.remove('disabled');
controlTab.innerHTML = 'Cal Motor  ✓';
} else {
controlTab.classList.add('disabled');
controlTab.innerHTML = 'Cal Motor ⚠️';
}

// Check if flows config exists
const hasFlows = flowCfg.flows && flowCfg.flows.length > 0;
if(hasFlows){
flowTab.classList.remove('disabled');
flowTab.innerHTML = 'Flow ✓';
} else {
flowTab.classList.add('disabled');
flowTab.innerHTML = 'Flow ⚠️';
}
}

connectWS();
setInterval(()=>{
if(ws&&ws.readyState===WebSocket.OPEN)ws.send('{"ping":1}');
},1000);
</script>
</body>
</html>
)=====";

DNSServer dnsServer;
AsyncWebServer server(80);

void setUpDNSServer(DNSServer &dnsServer, const IPAddress &localIP) {
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
  server.on("/connecttest.txt", [](AsyncWebServerRequest *request) { request->redirect("http://logout.net"); });
  server.on("/wpad.dat", [](AsyncWebServerRequest *request) { request->send(404); });
  server.on("/generate_204", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });
  server.on("/redirect", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });
  server.on("/hotspot-detect.html", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });
  server.on("/canonical.html", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });
  server.on("/success.txt", [](AsyncWebServerRequest *request) { request->send(200); });
  server.on("/ncsi.txt", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });
  server.on("/favicon.ico", [](AsyncWebServerRequest *request) { request->send(404); });

  server.on("/", HTTP_ANY, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  server.on("/config.json", HTTP_GET, [](AsyncWebServerRequest *request) {
    if(LittleFS.exists("/config.json")) {
      request->send(LittleFS, "/config.json", "application/json");
    } else {
      request->send(404, "text/plain", "Config not found");
    }
  });

  server.on("/save-config", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      static String configData = "";
      
      if(index == 0) {
        configData = "";
      }
      
      for(size_t i = 0; i < len; i++) {
        configData += (char)data[i];
      }
      
      if(index + len == total) {
        // Validate JSON
        DynamicJsonDocument doc(4096);
        DeserializationError error = deserializeJson(doc, configData);
        
        if(error) {
          request->send(400, "text/plain", "Invalid JSON");
          configData = "";
          return;
        }
        
        // Save to LittleFS
        File file = LittleFS.open("/config.json", "w");
        if(!file) {
          request->send(500, "text/plain", "Failed to save");
          configData = "";
          return;
        }
        
        file.print(configData);
        file.close();

        // Reload config vào RAM
        extern SystemConfig sysConfig;
        if(loadSystemConfig()) {
          request->send(200, "text/plain", "✅ Config saved & reloaded!");
          Serial.println("✅ Config reloaded");
        } else {
          request->send(500, "text/plain", "Saved but reload failed");
        }

        configData = "";
      }
    });
    server.on("/reload-motors", HTTP_GET, [](AsyncWebServerRequest *request) {
      // CHỈ reload config JSON, KHÔNG init lại hardware để tránh lỗi PCNT "Too many encoders"
      
      extern SystemConfig sysConfig;
      if(!loadSystemConfig()) {
        request->send(500, "text/plain", "Failed to load config");
        return;
      }
      
      // Cập nhật parameters từ config (KHÔNG touch encoder hardware)
      for(int i = 0; i < motorCount && i < sysConfig.motorCount; i++) {
        if(motors[i].initialized) {
          motors[i].name = sysConfig.motors[i].name;
          motors[i].pulsesPerRev = sysConfig.motors[i].pulses_per_rev;
          motors[i].degreesPerPulse = 360.0 / sysConfig.motors[i].pulses_per_rev;
          motors[i].pid.tune(sysConfig.motors[i].kp, sysConfig.motors[i].ki, sysConfig.motors[i].kd);
          motors[i].softLimitMin = sysConfig.motors[i].soft_min;
          motors[i].softLimitMax = sysConfig.motors[i].soft_max;
        }
      }
      
      request->send(200, "text/plain", "✅ Config reloaded (no hardware reinit)");
      Serial.println("✅ Config updated without hardware reinit");
    });
    server.on("/flow_config.json", HTTP_GET, [](AsyncWebServerRequest *request) {
    if(LittleFS.exists("/flow_config.json")) {
      request->send(LittleFS, "/flow_config.json", "application/json");
    } else {
      request->send(404, "text/plain", "Flow config not found");
    }
  });

  server.on("/save-flow-config", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      static String configData = "";
      
      if(index == 0) {
        configData = "";
      }
      
      for(size_t i = 0; i < len; i++) {
        configData += (char)data[i];
      }
      
      if(index + len == total) {
        DynamicJsonDocument doc(4096);
        DeserializationError error = deserializeJson(doc, configData);
        
        if(error) {
          request->send(400, "text/plain", "Invalid JSON");
          configData = "";
          return;
        }
        
        File file = LittleFS.open("/flow_config.json", "w");
        if(!file) {
          request->send(500, "text/plain", "Failed to save");
          configData = "";
          return;
        }
        
        file.print(configData);
        file.close();

        if(loadFlowConfig()) {
          request->send(200, "text/plain", "✅ Flow config saved!");
          Serial.println("✅ Flow config reloaded");
        } else {
          request->send(500, "text/plain", "Saved but reload failed");
        }

        configData = "";
      }
    });

    server.on("/reload-flows", HTTP_GET, [](AsyncWebServerRequest *request) {
      // Stop all flows
      for(int i = 0; i < flowSysConfig.flowCount; i++) {
        disableFlow(i);
      }
      
      delay(100);
      
      // Reload flow config
      if(!loadFlowConfig()) {
        request->send(500, "text/plain", "Failed to load flow config");
        return;
      }
      
      // Reinit flows
      initFlows();
      
      request->send(200, "text/plain", "✅ Flows reloaded!");
      Serial.println("✅ Flows reloaded from config");
    });

  server.onNotFound([](AsyncWebServerRequest *request) {
    request->redirect(localIPURL);
  });
  
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
}

void processCommandQueue() {
  if (!motorsReady) return;
  
  static unsigned long lastProcess = 0;
  
  if(!commandQueue.isEmpty() && millis() - lastProcess > 100) {
    lastProcess = millis();
    
    MotorCommandQueue::Command cmd = commandQueue.pop();
    int motorId = cmd.motorId;
    String command = cmd.cmd;
    command.toUpperCase();
    
    if(motorId >= motorCount) return;
    
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
  }
}

void broadcastStatusPeriodic() {
  static unsigned long lastBroadcast = 0;
  static String lastJson = "";
  
  if(millis() - lastBroadcast > 200) {
    lastBroadcast = millis();
    
    // Tạo JSON
    DynamicJsonDocument doc(2048);
    JsonArray motorsArray = doc.createNestedArray("motors");
    
    for (int i = 0; i < motorCount; i++) {
      if(!motors[i].initialized) continue;
      Motor &m = motors[i];
      JsonObject motor = motorsArray.createNestedObject();
      motor["id"] = m.id;
      motor["name"] = m.name;
      motor["position"] = (long)m.encoder.getCount();
      motor["angle"] = motorGetAngle(i);
      motor["rpm"] = m.motorRPM;
      motor["direction"] = (int)m.currentDir;
      motor["speed"] = (int)m.currentSpeed;
      motor["ppr"] = m.pulsesPerRev;
      
      JsonObject pid = motor.createNestedObject("pid");
      pid["kp"] = m.pid.Kp;
      pid["ki"] = m.pid.Ki;
      pid["kd"] = m.pid.Kd;
      pid["moves"] = m.pid.moveCount;
    }
    // Mode info
    doc["system_mode"] = (int)sysState.currentMode;
    doc["mode_name"] = getCurrentModeString();
    doc["status_msg"] = sysState.statusMessage;
    doc["flows_active"] = countActiveFlows();
    doc["queue"] = commandQueue.count;
    // ===== FLOW STATUS =====
    JsonArray flowsArray = doc.createNestedArray("flows");
    for(int i = 0; i < flowSysConfig.flowCount; i++) {
      JsonObject flow = flowsArray.createNestedObject();
      FlowConfigData* fc = &flowSysConfig.flows[i];
      FlowRuntime* fr = &flowRuntimes[i];
      
      flow["id"] = i;
      flow["name"] = fc->name;
      flow["type"] = fc->sensor.type;  // "ir" hoặc "touch"
      flow["motor_id"] = fc->motor_id;
      flow["active"] = fr->active;
      flow["state"] = (int)fr->state;  // 0=IDLE, 1=WAIT_CLEAR, 2=RECOVERY, 3=TOUCH_HOLD, 4=RELAY_ACTIVE
      
      // Sensor status
      if(fc->pins.sensor >= 0) {
        bool sensorDetected = (digitalRead(fc->pins.sensor) == LOW);
        flow["sensor_detected"] = sensorDetected;
      } else {
        flow["sensor_detected"] = false;
      }
      
      // Touch hold progress
      if(fc->sensor.type == "touch" && fr->state == 0 && fr->sensorLastState) {
        unsigned long holdDuration = millis() - fr->touchHoldStartTime;
        flow["hold_duration"] = holdDuration;
        flow["hold_required"] = fc->sensor.hold_time;
        if(holdDuration < fc->sensor.hold_time) {
          flow["hold_remaining"] = fc->sensor.hold_time - holdDuration;
        } else {
          flow["hold_remaining"] = 0;
        }
      } else {
        flow["hold_duration"] = 0;
        flow["hold_required"] = fc->sensor.hold_time;
        flow["hold_remaining"] = 0;
      }
      
      // Wait clear countdown (IR sensor)
      if(fc->sensor.type == "ir" && fr->state == 1) {  // FLOW_WAIT_CLEAR
        unsigned long elapsed = millis() - fr->sensorLastDetectedTime;
        if(elapsed < fc->sensor.clear_time) {
          flow["wait_remaining"] = fc->sensor.clear_time - elapsed;
        } else {
          flow["wait_remaining"] = 0;
        }
      } else {
        flow["wait_remaining"] = 0;
      }
      
      // Relay status (Touch sensor)
      if(fr->state == 4 && fr->relayStartTime > 0) {  // FLOW_RELAY_ACTIVE
        unsigned long elapsed = millis() - fr->relayStartTime;
        if(elapsed < fc->relay.duration) {
          flow["relay_active"] = true;
          flow["relay_remaining"] = fc->relay.duration - elapsed;
        } else {
          flow["relay_active"] = false;
          flow["relay_remaining"] = 0;
        }
      } else {
        flow["relay_active"] = false;
        flow["relay_remaining"] = 0;
      }
      
      // Limit switch status
      if(fc->pins.limit_cw >= 0) {
        flow["limit_cw"] = (digitalRead(fc->pins.limit_cw) == LOW);
      } else {
        flow["limit_cw"] = false;
      }
      
      if(fc->pins.limit_ccw >= 0) {
        flow["limit_ccw"] = (digitalRead(fc->pins.limit_ccw) == LOW);
      } else {
        flow["limit_ccw"] = false;
      }
      
      // Config info
      flow["angle"] = fc->movement.angle;
      flow["clear_time"] = fc->sensor.clear_time;
      flow["relay_duration"] = fc->relay.duration;
      if(fc->sensor.type == "completion") {
        flow["trigger_pending"] = fr->completionTriggerPending;        
        if(fr->completionTriggerPending && fr->completionTriggerTime > millis()) {
          unsigned long remaining = fr->completionTriggerTime - millis();
          flow["trigger_remaining"] = remaining;
        } else {
          flow["trigger_remaining"] = 0;
        }
      }

    // ===== RELAY TRIGGER WAITING STATUS (CHO TẤT CẢ FLOW) =====
    // NOTE: Cần thêm biến tracking trong FlowRuntime struct!
    // Tạm thời set false, cần implement logic chờ button sau
      if(fr->relayTriggerWaiting) {
        unsigned long elapsed = millis() - fr->relayTriggerWaitStart;
        flow["relay_trigger_waiting"] = true;
        flow["relay_trigger_elapsed"] = elapsed;
        flow["relay_trigger_remaining"] = (elapsed < 10000) ? (10000 - elapsed) : 0;
      } else {
        flow["relay_trigger_waiting"] = false;
        flow["relay_trigger_elapsed"] = 0;
        flow["relay_trigger_remaining"] = 0;
      }
    }
    // ===== END FLOW STATUS =====
    
    String currentJson;
    serializeJson(doc, currentJson);
    
    // Chỉ broadcast nếu có thay đổi
    if(currentJson != lastJson) {
      ws.textAll(currentJson);
      lastJson = currentJson;
    }
  }
}
void WifiPortalsetup() {
  Serial.println("\nMulti-Motor Control Portal");
  
  startSoftAccessPoint(ssid, password, localIP, gatewayIP);
  setUpDNSServer(dnsServer, localIP);
  setUpWebserver(server, localIP);
  server.begin();
  
  Serial.println("✓ Portal: " + String(ssid));
  Serial.println("✓ URL: " + localIPURL);
}

void WifiPortalloop() {
  dnsServer.processNextRequest();
  ws.cleanupClients();
  processCommandQueue();
  broadcastStatusPeriodic();
}

#endif
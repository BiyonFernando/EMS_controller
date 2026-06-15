#include "FesController.h"
#include <WiFi.h>

void handleRoot() {
  float currentVout = readOutputVoltage();
  float senseVoltage1 = hBridges[0].maxCurrent * CURRENT_SENSE_RESISTOR;
  float senseVoltage2 = hBridges[1].maxCurrent * CURRENT_SENSE_RESISTOR;
  float maxAllowedCurrent = getMaxAllowedCurrent();
  
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>ESP32 FES Control</title>";
  html += "<style>";
  html += "body { font-family: Arial; margin: 20px; background: #f0f0f0; }";
  html += ".container { max-width: 600px; margin: auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }";
  html += "h1 { color: #333; text-align: center; }";
  html += ".section { margin: 20px 0; padding: 15px; background: #f9f9f9; border-radius: 5px; }";
  html += ".label { font-weight: bold; margin-bottom: 5px; }";
  html += "input[type='number'] { width: 100%; padding: 8px; margin: 5px 0; box-sizing: border-box; }";
  html += "button { background: #4CAF50; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; margin: 5px; }";
  html += "button:hover { background: #45a049; }";
  html += ".btn-off { background: #f44336; }";
  html += ".btn-off:hover { background: #da190b; }";
  html += ".status { padding: 10px; background: #e3f2fd; border-radius: 5px; margin: 10px 0; }";
  html += ".value { color: #1976d2; font-weight: bold; }";
  html += ".warning { background: #fff3cd; color: #856404; }";
  html += "</style>";
  html += "<script>";
  html += "function updateValue(param, value) {";
  html += "  fetch('/set?param=' + param + '&value=' + value)";
  html += "    .then(response => response.text())";
  html += "    .then(data => { alert(data); location.reload(); });";
  html += "}";
  html += "function adjustVoltage(delta) {";
  html += "  var input = document.getElementById('voltage');";
  html += "  var current = parseFloat(input.value) || 0;";
  html += "  var next = Math.max(0, current + delta);";
  html += "  input.value = next.toFixed(1);";
  html += "  updateValue('voltage', input.value);";
  html += "}";
  html += "function updateStatus() {";
  html += "  fetch('/status')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      document.querySelectorAll('.status')[0].innerHTML = 'Target Voltage: <span class=\"value\">' + data.targetVoltage + ' V</span>';";
  html += "      document.querySelectorAll('.status')[1].innerHTML = 'Current Voltage: <span class=\"value\">' + data.currentVoltage + ' V</span>';";
  html += "      document.querySelectorAll('.status')[2].innerHTML = 'Duty Cycle: <span class=\"value\">' + data.currentDuty + ' %</span>';";
  html += "      document.querySelectorAll('.status')[3].innerHTML = 'Electrode 1: <span class=\"value\">' + (data.hBridge1Enabled ? 'ON' : 'OFF') + '</span> | Electrode 2: <span class=\"value\">' + (data.hBridge2Enabled ? 'ON' : 'OFF') + '</span>';";
  html += "      document.querySelectorAll('.status')[4].innerHTML = 'Electrode 1 Overcurrent Limit: <span class=\"value\">' + data.maxCurrent1 + ' A</span>';";
  html += "      document.querySelectorAll('.status')[5].innerHTML = 'Electrode 1 Comparator Ref: <span class=\"value\">' + data.senseVoltage1 + ' V</span>';";
  html += "      document.querySelectorAll('.status')[6].innerHTML = 'Electrode 2 Overcurrent Limit: <span class=\"value\">' + data.maxCurrent2 + ' A</span>';";
  html += "      document.querySelectorAll('.status')[7].innerHTML = 'Electrode 2 Comparator Ref: <span class=\"value\">' + data.senseVoltage2 + ' V</span>';";
  html += "      var oc1 = data.overcurrentProtection1 ? '<span style=\"color:red;\">ACTIVE (' + (data.timeSinceOvercurrent1/1000).toFixed(1) + 's)</span>' : '<span style=\"color:green;\">Normal</span>';";
  html += "      document.querySelectorAll('.status')[8].innerHTML = 'Electrode 1 Overcurrent: ' + oc1;";
  html += "      var oc2 = data.overcurrentProtection2 ? '<span style=\"color:red;\">ACTIVE (' + (data.timeSinceOvercurrent2/1000).toFixed(1) + 's)</span>' : '<span style=\"color:green;\">Normal</span>';";
  html += "      document.querySelectorAll('.status')[9].innerHTML = 'Electrode 2 Overcurrent: ' + oc2;";
  html += "      document.querySelectorAll('.status')[10].innerHTML = 'EMS Cycle: <span class=\"value\">' + (data.pulseCycleEnabled ? 'RUNNING' : 'STOPPED') + '</span> | Electrode 1 Cycle: <span class=\"value\">' + (data.electrode1CycleEnabled ? 'ON' : 'OFF') + '</span> | Electrode 2 Cycle: <span class=\"value\">' + (data.electrode2CycleEnabled ? 'ON' : 'OFF') + '</span>';";
  html += "      var pColor = data.sensor1High ? 'green' : 'orange';";
  html += "      document.querySelectorAll('.status')[11].innerHTML = 'Sensor 1 (GPIO2): <span style=\"color:' + pColor + ';font-weight:bold;\">' + (data.sensor1High ? 'HIGH' : 'LOW') + '</span> (' + data.sensor1Percent.toFixed(1) + '%)';";
  html += "      var s2Color = data.sensor2High ? 'green' : 'orange';";
  html += "      document.querySelectorAll('.status')[12].innerHTML = 'Sensor 2 (GPIO3): <span style=\"color:' + s2Color + ';font-weight:bold;\">' + (data.sensor2High ? 'HIGH' : 'LOW') + '</span> (' + data.sensor2Percent.toFixed(1) + '%)';";
  html += "      var smColor = data.sensorTriggerEnabled ? (data.smActive ? 'green' : 'blue') : 'gray';";
  html += "      document.querySelectorAll('.status')[13].innerHTML = 'Sensor Trigger SM: <span style=\"color:' + smColor + ';font-weight:bold;\">' + (data.sensorTriggerEnabled ? (data.smActive ? 'ACTIVE' : 'WAITING') : 'DISABLED') + '</span> | Max Time: <span class=\"value\">' + data.sensorTriggerMaxStimSeconds.toFixed(2) + ' s</span> | Electrode 1 Trigger: <span class=\"value\">' + (data.electrode1SensorTriggerEnabled ? 'ON' : 'OFF') + '</span> | Electrode 2 Trigger: <span class=\"value\">' + (data.electrode2SensorTriggerEnabled ? 'ON' : 'OFF') + '</span>';";
  html += "      document.querySelectorAll('.status')[14].innerHTML = 'Electrode 1 Pulse Width: <span class=\"value\">' + data.pulseWidth1 + ' us</span>';";
  html += "      document.querySelectorAll('.status')[15].innerHTML = 'Electrode 2 Pulse Width: <span class=\"value\">' + data.pulseWidth2 + ' us</span>';";
  html += "    });";  
  html += "}";
  html += "setInterval(updateStatus, 1000);";
  html += "</script>";
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h1>ESP32 FES Control Panel</h1>";
  
  html += "<div class='section'>";
  html += "<h2>Electrode Control</h2>";
  html += "<h3 style='margin: 10px 0;'>EMS Cycle Mode (0.55s ON / 1.05s OFF)</h3>";
  html += "<button onclick=\"location.href='/cycle/electrode1/on'\" style='background: #2196F3;'>Start Electrode 1 Cycle</button>";
  html += "<button onclick=\"location.href='/cycle/electrode2/on'\" style='background: #2196F3;'>Start Electrode 2 Cycle</button>";
  html += "<button onclick=\"location.href='/cycle/off'\" style='background: #FF9800;'>Stop EMS Cycle</button>";
  html += "<hr style='margin: 15px 0;'>";
  html += "<h3 style='margin: 10px 0;'>Sensor Trigger Mode</h3>";
  html += "<p style='font-size:0.9em;color:#555;margin:5px 0;'>Pulses ON at Sensor1 (GPIO2) falling edge &rarr; OFF at Sensor2 (GPIO3) rising edge</p>";
  html += "<div class='label'>Maximum Stimulation Time (s): <span class='value'>" + String(SENSOR_TRIGGER_MAX_STIM_SECONDS, 2) + "</span></div>";
  html += "<input type='number' id='sensorMaxStim' value='" + String(SENSOR_TRIGGER_MAX_STIM_SECONDS, 2) + "' step='0.05' min='0.05'>";
  html += "<button onclick=\"updateValue('sensorMaxStim', document.getElementById('sensorMaxStim').value)\">Set Max Time</button>";
  html += "<button onclick=\"location.href='/sensor/electrode1/on'\" style='background: #9C27B0;'>Enable Electrode 1 Sensor Trigger</button>";
  html += "<button onclick=\"location.href='/sensor/electrode2/on'\" style='background: #9C27B0;'>Enable Electrode 2 Sensor Trigger</button>";
  html += "<button onclick=\"location.href='/sensor/off'\" style='background: #607D8B;'>Disable Sensor Trigger</button>";
  html += "</div>";
  
  html += "<div class='section'>";
  html += "<h2>Current Status</h2>";
  html += "<div class='status'>Target Voltage: <span class='value'>" + String(TARGET_VOLTAGE, 1) + " V</span></div>";
  html += "<div class='status'>Current Voltage: <span class='value'>" + String(currentVout, 1) + " V</span></div>";
  html += "<div class='status'>Duty Cycle: <span class='value'>" + String(currentDuty * 100.0, 2) + " %</span></div>";
  html += "<div class='status'>Electrode 1: <span class='value'>" + String(hBridges[0].enabled ? "ON" : "OFF") + "</span> | Electrode 2: <span class='value'>" + String(hBridges[1].enabled ? "ON" : "OFF") + "</span></div>";
  html += "<div class='status warning'>Electrode 1 Overcurrent Limit: <span class='value'>" + String(hBridges[0].maxCurrent, 3) + " A</span></div>";
  html += "<div class='status warning'>Electrode 1 Comparator Ref: <span class='value'>" + String(senseVoltage1, 3) + " V</span></div>";
  html += "<div class='status warning'>Electrode 2 Overcurrent Limit: <span class='value'>" + String(hBridges[1].maxCurrent, 3) + " A</span></div>";
  html += "<div class='status warning'>Electrode 2 Comparator Ref: <span class='value'>" + String(senseVoltage2, 3) + " V</span></div>";

  String oc1Status = hBridges[0].overcurrentProtection ? "<span style='color:red;'>ACTIVE</span>" : "<span style='color:green;'>Normal</span>";
  html += "<div class='status warning'>Electrode 1 Overcurrent: " + oc1Status + "</div>";
  String oc2Status = hBridges[1].overcurrentProtection ? "<span style='color:red;'>ACTIVE</span>" : "<span style='color:green;'>Normal</span>";
  html += "<div class='status warning'>Electrode 2 Overcurrent: " + oc2Status + "</div>";
  html += "<div class='status'>EMS Cycle: <span class='value'>" + String(pulseCycleEnabled ? "RUNNING" : "STOPPED") + "</span> | Electrode 1 Cycle: <span class='value'>" + String(electrodeCycleEnabled[0] ? "ON" : "OFF") + "</span> | Electrode 2 Cycle: <span class='value'>" + String(electrodeCycleEnabled[1] ? "ON" : "OFF") + "</span></div>";
  String pColor = sensor1High ? "green" : "orange";
  html += "<div class='status'>Sensor 1 (GPIO2): <span style='color:" + pColor + ";font-weight:bold;'>" + String(sensor1High ? "HIGH" : "LOW") + "</span> (" + String(sensor1Percent, 1) + "%)</div>";
  String s2Color = sensor2High ? "green" : "orange";
  html += "<div class='status'>Sensor 2 (GPIO3): <span style='color:" + s2Color + ";font-weight:bold;'>" + String(sensor2High ? "HIGH" : "LOW") + "</span> (" + String(sensor2Percent, 1) + "%)</div>";
  String smColor = sensorTriggerEnabled ? (sensorSmState == SM_ACTIVE ? "green" : "blue") : "gray";
  String smLabel = sensorTriggerEnabled ? (sensorSmState == SM_ACTIVE ? "ACTIVE" : "WAITING") : "DISABLED";
  html += "<div class='status'>Sensor Trigger SM: <span style='color:" + smColor + ";font-weight:bold;'>" + smLabel + "</span> | Max Time: <span class='value'>" + String(SENSOR_TRIGGER_MAX_STIM_SECONDS, 2) + " s</span> | Electrode 1 Trigger: <span class='value'>" + String(electrodeSensorTriggerEnabled[0] ? "ON" : "OFF") + "</span> | Electrode 2 Trigger: <span class='value'>" + String(electrodeSensorTriggerEnabled[1] ? "ON" : "OFF") + "</span></div>";
  html += "<div class='status'>Electrode 1 Pulse Width: <span class='value'>" + String(hBridges[0].pulseWidthUs) + " us</span></div>";
  html += "<div class='status'>Electrode 2 Pulse Width: <span class='value'>" + String(hBridges[1].pulseWidthUs) + " us</span></div>";
  html += "</div>";
  
  html += "<div class='section'>";
  html += "<h2>Voltage Control</h2>";
  html += "<div class='label'>Target Voltage (V):</div>";
  html += "<input type='number' id='voltage' value='" + String(TARGET_VOLTAGE, 1) + "' step='1' min='0'>";
  html += "<button onclick=\"adjustVoltage(2.5)\" style='background: #2196F3;'>+2.5 V</button>";
  html += "<button onclick=\"adjustVoltage(-2.5)\" style='background: #607D8B;'>-2.5 V</button>";
  html += "<button onclick=\"updateValue('voltage', document.getElementById('voltage').value)\">Set Voltage</button>";
  html += "</div>";
  
  html += "<div class='section'>";
  html += "<h2>Overcurrent Limit Control</h2>";
  html += "<div class='label'>Electrode 1 Overcurrent Limit (A): <span class='value'>" + String(hBridges[0].maxCurrent, 3) + "</span></div>";
  html += "<input type='number' id='current1' value='" + String(hBridges[0].maxCurrent, 3) + "' step='0.01' min='0' max='" + String(maxAllowedCurrent, 3) + "'>";
  html += "<button onclick=\"updateValue('current1', document.getElementById('current1').value)\">Set Electrode 1 Limit</button>";
  html += "<div class='label'>Electrode 2 Overcurrent Limit (A): <span class='value'>" + String(hBridges[1].maxCurrent, 3) + "</span></div>";
  html += "<input type='number' id='current2' value='" + String(hBridges[1].maxCurrent, 3) + "' step='0.01' min='0' max='" + String(maxAllowedCurrent, 3) + "'>";
  html += "<button onclick=\"updateValue('current2', document.getElementById('current2').value)\">Set Electrode 2 Limit</button>";
  html += "<div style='margin-top: 10px; font-size: 0.9em; color: #666;'>";
  html += "I_MAX_1 = GPIO16, I_MAX_2 = GPIO18 (PWM comparator reference)<br>";
  html += "Current sense resistor: " + String(CURRENT_SENSE_RESISTOR, 1) + " Ohm<br>";
  html += "Electrode 1 ref: " + String(senseVoltage1, 3) + "V, Electrode 2 ref: " + String(senseVoltage2, 3) + "V<br>";
  html += "<strong>Max allowed (95% duty): " + String(maxAllowedCurrent, 3) + "A per channel</strong>";
  html += "</div>";
  html += "</div>";

  html += "<div class='section'>";
  html += "<h2>Pulse Width Control</h2>";
  html += "<div class='label'>Electrode 1 Pulse Width (us): <span class='value'>" + String(hBridges[0].pulseWidthUs) + "</span></div>";
  html += "<input type='number' id='width1' value='" + String(hBridges[0].pulseWidthUs) + "' step='10' min='" + String(PULSE_WIDTH_MIN_US) + "' max='" + String(PULSE_WIDTH_MAX_US) + "'>";
  html += "<button onclick=\"updateValue('width1', document.getElementById('width1').value)\">Set Electrode 1 Width</button>";
  html += "<div class='label'>Electrode 2 Pulse Width (us): <span class='value'>" + String(hBridges[1].pulseWidthUs) + "</span></div>";
  html += "<input type='number' id='width2' value='" + String(hBridges[1].pulseWidthUs) + "' step='10' min='" + String(PULSE_WIDTH_MIN_US) + "' max='" + String(PULSE_WIDTH_MAX_US) + "'>";
  html += "<button onclick=\"updateValue('width2', document.getElementById('width2').value)\">Set Electrode 2 Width</button>";
  html += "<div style='margin-top: 10px; font-size: 0.9em; color: #666;'>";
  html += "Period: " + String(PULSE_PERIOD_US) + " us, Gap: " + String(PULSE_GAP_US) + " us<br>";
  html += "Max width per bridge: " + String(PULSE_WIDTH_MAX_US) + " us";
  html += "</div>";
  html += "</div>";

  html += "<div class='section'>";
  html += "<h2>Sensor Threshold Control</h2>";
  html += "<div class='label'>Sensor 1 Threshold (%): <span class='value'>" + String(SENSOR1_THRESHOLD_PERCENT, 1) + "</span></div>";
  html += "<input type='number' id='threshold1' value='" + String(SENSOR1_THRESHOLD_PERCENT, 1) + "' step='1' min='0' max='100'>";
  html += "<button onclick=\"updateValue('threshold1', document.getElementById('threshold1').value)\">Set Sensor 1 Threshold</button>";

  html += "<div class='label'>Sensor 2 Threshold (%): <span class='value'>" + String(SENSOR2_THRESHOLD_PERCENT, 1) + "</span></div>";
  html += "<input type='number' id='threshold2' value='" + String(SENSOR2_THRESHOLD_PERCENT, 1) + "' step='1' min='0' max='100'>";
  html += "<button onclick=\"updateValue('threshold2', document.getElementById('threshold2').value)\">Set Sensor 2 Threshold</button>";
  html += "</div>";

  html += "</div></body></html>";
  
  server.send(200, "text/html", html);
}

void handleCycleElectrode1On() {
  startElectrodeCycle(0);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleCycleElectrode2On() {
  startElectrodeCycle(1);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleCycleOff() {
  portENTER_CRITICAL(&timerMux);
  pulseCycleEnabled = false;
  electrodeCycleEnabled[0] = false;
  electrodeCycleEnabled[1] = false;
  cyclePhaseOn = false;
  pulseOutputEnabled = false;
  hBridges[0].enabled = false;
  hBridges[1].enabled = false;
  hBridges[0].lastPulseState = -1;
  hBridges[1].lastPulseState = -1;
  digitalWrite(hBridges[0].posPin, LOW);
  digitalWrite(hBridges[0].negPin, LOW);
  digitalWrite(hBridges[1].posPin, LOW);
  digitalWrite(hBridges[1].negPin, LOW);
  portEXIT_CRITICAL(&timerMux);

  Serial.println("Web: EMS cycle stopped");
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSensorTriggerElectrode1On() {
  startElectrodeSensorTrigger(0);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSensorTriggerElectrode2On() {
  startElectrodeSensorTrigger(1);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSensorTriggerOn() {
  startElectrodeSensorTrigger(0);
  startElectrodeSensorTrigger(1);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSensorTriggerOff() {
  portENTER_CRITICAL(&timerMux);
  for (int i = 0; i < NUM_HBRIDGES; i++) {
    if (electrodeSensorTriggerEnabled[i]) {
      hBridges[i].enabled = false;
      hBridges[i].lastPulseState = -1;
      digitalWrite(hBridges[i].posPin, LOW);
      digitalWrite(hBridges[i].negPin, LOW);
    }
  }
  sensorTriggerEnabled = false;
  electrodeSensorTriggerEnabled[0] = false;
  electrodeSensorTriggerEnabled[1] = false;
  sensorSmState = SM_WAITING;
  sensorTriggerStartTime = 0;
  pulseOutputEnabled = hBridges[0].enabled || hBridges[1].enabled;
  portEXIT_CRITICAL(&timerMux);

  Serial.println("Web: Sensor trigger mode DISABLED");
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSet() {
  if (server.hasArg("param") && server.hasArg("value")) {
    String param = server.arg("param");
    float value = server.arg("value").toFloat();
    
    if (param == "voltage") {
      if (value < 0.0f) value = 0.0f;
      TARGET_VOLTAGE = value;
      Serial.println("Set target voltage to " + String(value, 1) + "V");
      server.send(200, "text/plain", "Voltage set to " + String(value, 1) + "V");
    }
    else if (param == "current1") {
      setMaxCurrent(0, value);
      server.send(200, "text/plain", "Electrode 1 overcurrent limit set to " + String(hBridges[0].maxCurrent, 3) + "A");
    }
    else if (param == "current2") {
      setMaxCurrent(1, value);
      server.send(200, "text/plain", "Electrode 2 overcurrent limit set to " + String(hBridges[1].maxCurrent, 3) + "A");
    }
    else if (param == "width1") {
      setPulseWidth(0, (unsigned long)value);
      server.send(200, "text/plain", "Electrode 1 pulse width set to " + String(hBridges[0].pulseWidthUs) + " us");
    }
    else if (param == "width2") {
      setPulseWidth(1, (unsigned long)value);
      server.send(200, "text/plain", "Electrode 2 pulse width set to " + String(hBridges[1].pulseWidthUs) + " us");
    }
    else if (param == "threshold1") {
      if (value < 0.0f) value = 0.0f;
      if (value > 100.0f) value = 100.0f;
      SENSOR1_THRESHOLD_PERCENT = value;
      Serial.println("Set Sensor 1 threshold to " + String(value, 1) + "%");
      server.send(200, "text/plain", "Sensor 1 threshold set to " + String(value, 1) + "%");
    }
    else if (param == "threshold2") {
      if (value < 0.0f) value = 0.0f;
      if (value > 100.0f) value = 100.0f;
      SENSOR2_THRESHOLD_PERCENT = value;
      Serial.println("Set Sensor 2 threshold to " + String(value, 1) + "%");
      server.send(200, "text/plain", "Sensor 2 threshold set to " + String(value, 1) + "%");
    }
    else if (param == "sensorMaxStim") {
      if (value < 0.05f) value = 0.05f;
      SENSOR_TRIGGER_MAX_STIM_SECONDS = value;
      Serial.println("Set sensor trigger max stimulation time to " + String(value, 2) + "s");
      server.send(200, "text/plain", "Sensor trigger max stimulation time set to " + String(value, 2) + "s");
    }
    else {
      server.send(400, "text/plain", "Unknown parameter");
    }
  } else {
    server.send(400, "text/plain", "Missing parameters");
  }
}

void handleStatus() {
  float senseVoltage1 = hBridges[0].maxCurrent * CURRENT_SENSE_RESISTOR;
  float senseVoltage2 = hBridges[1].maxCurrent * CURRENT_SENSE_RESISTOR;
  float maxAllowedCurrent = getMaxAllowedCurrent();
  unsigned long timeSinceOvercurrent1 = 0;
  unsigned long timeSinceOvercurrent2 = 0;

  if (hBridges[0].overcurrentProtection && hBridges[0].overcurrentStartTime > 0) {
    timeSinceOvercurrent1 = millis() - hBridges[0].overcurrentStartTime;
  }
  if (hBridges[1].overcurrentProtection && hBridges[1].overcurrentStartTime > 0) {
    timeSinceOvercurrent2 = millis() - hBridges[1].overcurrentStartTime;
  }

  String json = "{";
  json += "\"pulseEnabled\":" + String(pulseOutputEnabled ? "true" : "false") + ",";
  json += "\"hBridge1Enabled\":" + String(hBridges[0].enabled ? "true" : "false") + ",";
  json += "\"hBridge2Enabled\":" + String(hBridges[1].enabled ? "true" : "false") + ",";
  json += "\"targetVoltage\":" + String(TARGET_VOLTAGE, 1) + ",";
  json += "\"currentVoltage\":" + String(readOutputVoltage(), 1) + ",";
  json += "\"currentDuty\":" + String(currentDuty * 100, 2) + ",";
  json += "\"maxCurrent1\":" + String(hBridges[0].maxCurrent, 3) + ",";
  json += "\"maxCurrent2\":" + String(hBridges[1].maxCurrent, 3) + ",";
  json += "\"senseVoltage1\":" + String(senseVoltage1, 3) + ",";
  json += "\"senseVoltage2\":" + String(senseVoltage2, 3) + ",";
  json += "\"maxAllowedCurrent\":" + String(maxAllowedCurrent, 3) + ",";
  json += "\"overcurrentProtection1\":" + String(hBridges[0].overcurrentProtection ? "true" : "false") + ",";
  json += "\"timeSinceOvercurrent1\":" + String(timeSinceOvercurrent1) + ",";
  json += "\"overcurrentProtection2\":" + String(hBridges[1].overcurrentProtection ? "true" : "false") + ",";
  json += "\"timeSinceOvercurrent2\":" + String(timeSinceOvercurrent2) + ",";
  json += "\"pulseCycleEnabled\":" + String(pulseCycleEnabled ? "true" : "false") + ",";
  json += "\"electrode1CycleEnabled\":" + String(electrodeCycleEnabled[0] ? "true" : "false") + ",";
  json += "\"electrode2CycleEnabled\":" + String(electrodeCycleEnabled[1] ? "true" : "false") + ",";
  json += "\"sensor1Percent\":" + String(sensor1Percent, 1) + ",";
  json += "\"sensor1High\":" + String(sensor1High ? "true" : "false") + ",";
  json += "\"sensor2Percent\":" + String(sensor2Percent, 1) + ",";
  json += "\"sensor2High\":" + String(sensor2High ? "true" : "false") + ",";
  json += "\"sensorTriggerEnabled\":" + String(sensorTriggerEnabled ? "true" : "false") + ",";
  json += "\"electrode1SensorTriggerEnabled\":" + String(electrodeSensorTriggerEnabled[0] ? "true" : "false") + ",";
  json += "\"electrode2SensorTriggerEnabled\":" + String(electrodeSensorTriggerEnabled[1] ? "true" : "false") + ",";
  json += "\"sensorTriggerMaxStimSeconds\":" + String(SENSOR_TRIGGER_MAX_STIM_SECONDS, 2) + ",";
  json += "\"smActive\":" + String(sensorSmState == SM_ACTIVE ? "true" : "false") + ",";
  json += "\"pulseWidth1\":" + String(hBridges[0].pulseWidthUs) + ",";
  json += "\"pulseWidth2\":" + String(hBridges[1].pulseWidthUs);
  json += "}";

  server.send(200, "application/json", json);
}

void setupWebServer()
{
  Serial.println("Setting up WiFi Access Point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.println("Connect to the AP and navigate to: http://" + IP.toString());

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/set", handleSet);
  server.on("/cycle/electrode1/on", handleCycleElectrode1On);
  server.on("/cycle/electrode2/on", handleCycleElectrode2On);
  server.on("/cycle/off", handleCycleOff);
  server.on("/sensor/on", handleSensorTriggerOn);
  server.on("/sensor/electrode1/on", handleSensorTriggerElectrode1On);
  server.on("/sensor/electrode2/on", handleSensorTriggerElectrode2On);
  server.on("/sensor/off", handleSensorTriggerOff);
  
  server.begin();
  Serial.println("Web server started");
}

void updateWebServer()
{
  server.handleClient();
}

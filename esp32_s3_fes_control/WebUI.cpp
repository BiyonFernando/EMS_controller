#include "FesController.h"
#include <WiFi.h>

static String getElectrodeSensorState(int bridgeIndex);
static String getTriggerModeLabel();

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
  html += "var sensorLogRows = [];";
  html += "function collectSensorLogs() {";
  html += "  fetch('/sensor/log')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      (data.entries || []).forEach(entry => sensorLogRows.push(entry));";
  html += "    })";
  html += "    .catch(() => {});";
  html += "}";
  html += "function pad2(value) {";
  html += "  return String(value).padStart(2, '0');";
  html += "}";
  html += "function formatTimestampForFilename(date) {";
  html += "  var y = date.getFullYear();";
  html += "  var m = pad2(date.getMonth() + 1);";
  html += "  var d = pad2(date.getDate());";
  html += "  var h = pad2(date.getHours());";
  html += "  var min = pad2(date.getMinutes());";
  html += "  var s = pad2(date.getSeconds());";
  html += "  return y + m + d + '_' + h + min + s;";
  html += "}";
  html += "function buildElectrodeConfig(electrodeNumber, status) {";
  html += "  var prefix = electrodeNumber == 1 ? 'e1' : 'e2';";
  html += "  var events = '';";
  html += "  if (status[prefix + 'Sensor1Falling']) events += 'F1';";
  html += "  if (status[prefix + 'Sensor1Rising']) events += 'R1';";
  html += "  if (status[prefix + 'Sensor2Falling']) events += 'F2';";
  html += "  if (status[prefix + 'Sensor2Rising']) events += 'R2';";
  html += "  if (events.length == 0) events = 'None';";
  html += "  var stimSeconds = electrodeNumber == 1 ? status.electrode1StimDuration : status.electrode2StimDuration;";
  html += "  var silentSeconds = electrodeNumber == 1 ? status.electrode1SilentPeriod : status.electrode2SilentPeriod;";
  html += "  var stimMs = Math.round((parseFloat(stimSeconds) || 0) * 1000);";
  html += "  var silentMs = Math.round((parseFloat(silentSeconds) || 0) * 1000);";
  html += "  return 'E' + electrodeNumber + '-' + events + '-' + stimMs + '-' + silentMs;";
  html += "}";
  html += "function buildSensorLogFilename(status) {";
  html += "  return 'sensor_log_' + formatTimestampForFilename(new Date()) + '_' + buildElectrodeConfig(1, status) + '_' + buildElectrodeConfig(2, status) + '.csv';";
  html += "}";
  html += "function downloadSensorCsv(filename) {";
  html += "  var csv = 'timestamp,event,triggered\\n';";
  html += "  sensorLogRows.forEach(row => {";
  html += "    csv += row.timestamp + ',' + row.event + ',' + row.triggered + '\\n';";
  html += "  });";
  html += "  var blob = new Blob([csv], {type: 'text/csv'});";
  html += "  var link = document.createElement('a');";
  html += "  link.href = URL.createObjectURL(blob);";
  html += "  link.download = filename;";
  html += "  document.body.appendChild(link);";
  html += "  link.click();";
  html += "  document.body.removeChild(link);";
  html += "  URL.revokeObjectURL(link.href);";
  html += "}";
  html += "function disableSensorTrigger() {";
  html += "  Promise.all([fetch('/sensor/log').then(response => response.json()), fetch('/status').then(response => response.json())])";
  html += "    .then(results => {";
  html += "      var logData = results[0];";
  html += "      var status = results[1];";
  html += "      (logData.entries || []).forEach(entry => sensorLogRows.push(entry));";
  html += "      var filename = buildSensorLogFilename(status);";
  html += "      fetch('/sensor/off')";
  html += "        .then(() => { downloadSensorCsv(filename); location.reload(); })";
  html += "        .catch(() => { downloadSensorCsv(filename); location.reload(); });";
  html += "    })";
  html += "    .catch(() => {";
  html += "      var fallback = {e1Sensor1Falling:false,e1Sensor1Rising:false,e1Sensor2Falling:false,e1Sensor2Rising:false,e2Sensor1Falling:false,e2Sensor1Rising:false,e2Sensor2Falling:false,e2Sensor2Rising:false,electrode1StimDuration:0,electrode1SilentPeriod:0,electrode2StimDuration:0,electrode2SilentPeriod:0};";
  html += "      var filename = buildSensorLogFilename(fallback);";
  html += "      fetch('/sensor/off')";
  html += "        .then(() => { downloadSensorCsv(filename); location.reload(); })";
  html += "        .catch(() => { downloadSensorCsv(filename); location.reload(); });";
  html += "    });";
  html += "}";
  html += "function updateValue(param, value) {";
  html += "  fetch('/set?param=' + param + '&value=' + value)";
  html += "    .then(response => response.text())";
  html += "    .then(data => { alert(data); location.reload(); });";
  html += "}";
  html += "function updateCheckbox(param, checked) {";
  html += "  fetch('/set?param=' + param + '&value=' + (checked ? '1' : '0'))";
  html += "    .then(response => response.text())";
  html += "    .then(data => { updateStatus(); });";
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
  html += "      document.querySelectorAll('.status')[10].innerHTML = 'Trigger Mode: <span class=\"value\">' + data.triggerMode + '</span> | Electrode 1 Cycle: <span class=\"value\">' + (data.electrode1CycleEnabled ? 'ON' : 'OFF') + '</span> | Electrode 2 Cycle: <span class=\"value\">' + (data.electrode2CycleEnabled ? 'ON' : 'OFF') + '</span>';";
  html += "      var pColor = data.sensor1High ? 'green' : 'orange';";
  html += "      document.querySelectorAll('.status')[11].innerHTML = 'Sensor 1 (GPIO2): <span style=\"color:' + pColor + ';font-weight:bold;\">' + (data.sensor1High ? 'HIGH' : 'LOW') + '</span> (' + data.sensor1Percent.toFixed(1) + '%)';";
  html += "      var s2Color = data.sensor2High ? 'green' : 'orange';";
  html += "      document.querySelectorAll('.status')[12].innerHTML = 'Sensor 2 (GPIO3): <span style=\"color:' + s2Color + ';font-weight:bold;\">' + (data.sensor2High ? 'HIGH' : 'LOW') + '</span> (' + data.sensor2Percent.toFixed(1) + '%)';";
  html += "      document.querySelectorAll('.status')[13].innerHTML = 'Sensor Trigger: <span class=\"value\">' + (data.sensorTriggerEnabled ? 'ENABLED' : 'DISABLED') + '</span> | Electrode 1: <span class=\"value\">' + data.electrode1SensorState + '</span> | Electrode 2: <span class=\"value\">' + data.electrode2SensorState + '</span>';";
  html += "      document.querySelectorAll('.status')[14].innerHTML = 'Electrode 1 Pulse Width: <span class=\"value\">' + data.pulseWidth1 + ' us</span>';";
  html += "      document.querySelectorAll('.status')[15].innerHTML = 'Electrode 2 Pulse Width: <span class=\"value\">' + data.pulseWidth2 + ' us</span>';";
  html += "      document.querySelectorAll('.status')[16].innerHTML = 'Boost Kp: <span class=\"value\">' + data.kp.toFixed(6) + '</span> | Kd: <span class=\"value\">' + data.kd.toFixed(6) + '</span>';";
  html += "    });";  
  html += "}";
  html += "setInterval(updateStatus, 1000);";
  html += "setInterval(collectSensorLogs, 500);";
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
  html += "<div style='padding:10px 0;'>";
  html += "<h4 style='margin:8px 0;'>Electrode 1 Trigger Events</h4>";
  html += "<label><input type='checkbox' onchange=\"updateCheckbox('e1_s1_fall', this.checked)\" " + String(electrodeTriggerEvents[0][SENSOR1_FALLING] ? "checked" : "") + "> Sensor 1 falling edge</label><br>";
  html += "<label><input type='checkbox' onchange=\"updateCheckbox('e1_s2_fall', this.checked)\" " + String(electrodeTriggerEvents[0][SENSOR2_FALLING] ? "checked" : "") + "> Sensor 2 falling edge</label><br>";
  html += "<label><input type='checkbox' onchange=\"updateCheckbox('e1_s1_rise', this.checked)\" " + String(electrodeTriggerEvents[0][SENSOR1_RISING] ? "checked" : "") + "> Sensor 1 rising edge</label><br>";
  html += "<label><input type='checkbox' onchange=\"updateCheckbox('e1_s2_rise', this.checked)\" " + String(electrodeTriggerEvents[0][SENSOR2_RISING] ? "checked" : "") + "> Sensor 2 rising edge</label>";
  html += "<div class='label'>Stimulation Duration (s): <span class='value'>" + String(electrodeStimDurationSeconds[0], 2) + "</span></div>";
  html += "<input type='number' id='e1StimDuration' value='" + String(electrodeStimDurationSeconds[0], 2) + "' step='0.05' min='0.01'>";
  html += "<button onclick=\"updateValue('e1StimDuration', document.getElementById('e1StimDuration').value)\">Set Electrode 1 Duration</button>";
  html += "<div class='label'>Silent Period (s): <span class='value'>" + String(electrodeSilentSeconds[0], 2) + "</span></div>";
  html += "<input type='number' id='e1SilentPeriod' value='" + String(electrodeSilentSeconds[0], 2) + "' step='0.05' min='0'>";
  html += "<button onclick=\"updateValue('e1SilentPeriod', document.getElementById('e1SilentPeriod').value)\">Set Electrode 1 Silent Period</button>";
  html += "<button onclick=\"location.href='/sensor/electrode1/start'\" style='background: #9C27B0;'>Start Electrode 1 Sensor Trigger</button>";
  html += "</div>";
  html += "<div style='padding:10px 0;'>";
  html += "<h4 style='margin:8px 0;'>Electrode 2 Trigger Events</h4>";
  html += "<label><input type='checkbox' onchange=\"updateCheckbox('e2_s1_fall', this.checked)\" " + String(electrodeTriggerEvents[1][SENSOR1_FALLING] ? "checked" : "") + "> Sensor 1 falling edge</label><br>";
  html += "<label><input type='checkbox' onchange=\"updateCheckbox('e2_s2_fall', this.checked)\" " + String(electrodeTriggerEvents[1][SENSOR2_FALLING] ? "checked" : "") + "> Sensor 2 falling edge</label><br>";
  html += "<label><input type='checkbox' onchange=\"updateCheckbox('e2_s1_rise', this.checked)\" " + String(electrodeTriggerEvents[1][SENSOR1_RISING] ? "checked" : "") + "> Sensor 1 rising edge</label><br>";
  html += "<label><input type='checkbox' onchange=\"updateCheckbox('e2_s2_rise', this.checked)\" " + String(electrodeTriggerEvents[1][SENSOR2_RISING] ? "checked" : "") + "> Sensor 2 rising edge</label>";
  html += "<div class='label'>Stimulation Duration (s): <span class='value'>" + String(electrodeStimDurationSeconds[1], 2) + "</span></div>";
  html += "<input type='number' id='e2StimDuration' value='" + String(electrodeStimDurationSeconds[1], 2) + "' step='0.05' min='0.01'>";
  html += "<button onclick=\"updateValue('e2StimDuration', document.getElementById('e2StimDuration').value)\">Set Electrode 2 Duration</button>";
  html += "<div class='label'>Silent Period (s): <span class='value'>" + String(electrodeSilentSeconds[1], 2) + "</span></div>";
  html += "<input type='number' id='e2SilentPeriod' value='" + String(electrodeSilentSeconds[1], 2) + "' step='0.05' min='0'>";
  html += "<button onclick=\"updateValue('e2SilentPeriod', document.getElementById('e2SilentPeriod').value)\">Set Electrode 2 Silent Period</button>";
  html += "<button onclick=\"location.href='/sensor/electrode2/start'\" style='background: #9C27B0;'>Start Electrode 2 Sensor Trigger</button>";
  html += "</div>";
  html += "<button onclick=\"disableSensorTrigger()\" style='background: #607D8B;'>Disable Sensor Trigger</button>";
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
  html += "<div class='status'>Trigger Mode: <span class='value'>" + getTriggerModeLabel() + "</span> | Electrode 1 Cycle: <span class='value'>" + String(electrodeCycleEnabled[0] ? "ON" : "OFF") + "</span> | Electrode 2 Cycle: <span class='value'>" + String(electrodeCycleEnabled[1] ? "ON" : "OFF") + "</span></div>";
  String pColor = sensor1High ? "green" : "orange";
  html += "<div class='status'>Sensor 1 (GPIO2): <span style='color:" + pColor + ";font-weight:bold;'>" + String(sensor1High ? "HIGH" : "LOW") + "</span> (" + String(sensor1Percent, 1) + "%)</div>";
  String s2Color = sensor2High ? "green" : "orange";
  html += "<div class='status'>Sensor 2 (GPIO3): <span style='color:" + s2Color + ";font-weight:bold;'>" + String(sensor2High ? "HIGH" : "LOW") + "</span> (" + String(sensor2Percent, 1) + "%)</div>";
  String e1SensorState = getElectrodeSensorState(0);
  String e2SensorState = getElectrodeSensorState(1);
  html += "<div class='status'>Sensor Trigger: <span class='value'>" + String(sensorTriggerEnabled ? "ENABLED" : "DISABLED") + "</span> | Electrode 1: <span class='value'>" + e1SensorState + "</span> | Electrode 2: <span class='value'>" + e2SensorState + "</span></div>";
  html += "<div class='status'>Electrode 1 Pulse Width: <span class='value'>" + String(hBridges[0].pulseWidthUs) + " us</span></div>";
  html += "<div class='status'>Electrode 2 Pulse Width: <span class='value'>" + String(hBridges[1].pulseWidthUs) + " us</span></div>";
  html += "<div class='status'>Boost Kp: <span class='value'>" + String(Kp, 6) + "</span> | Kd: <span class='value'>" + String(Kd, 6) + "</span></div>";
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

  html += "<div class='section'>";
  html += "<h2>Boost Controller Tuning</h2>";
  html += "<div class='label'>Kp: <span class='value'>" + String(Kp, 6) + "</span></div>";
  html += "<input type='number' id='kp' value='" + String(Kp, 6) + "' step='0.0001' min='0'>";
  html += "<button onclick=\"updateValue('kp', document.getElementById('kp').value)\">Set Kp</button>";
  html += "<div class='label'>Kd: <span class='value'>" + String(Kd, 6) + "</span></div>";
  html += "<input type='number' id='kd' value='" + String(Kd, 6) + "' step='0.0001' min='0'>";
  html += "<button onclick=\"updateValue('kd', document.getElementById('kd').value)\">Set Kd</button>";
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
  stopAllTriggering();
  Serial.println("Web: EMS cycle stopped");
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSensorTriggerOff() {
  stopAllSensorTriggers();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSensorTriggerElectrode1Start() {
  startElectrodeSensorTrigger(0);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSensorTriggerElectrode2Start() {
  startElectrodeSensorTrigger(1);
  server.sendHeader("Location", "/");
  server.send(303);
}

static bool anySelectedTriggerForElectrode(int bridgeIndex)
{
  for (int i = 0; i < SENSOR_TRIGGER_EVENT_COUNT; i++) {
    if (electrodeTriggerEvents[bridgeIndex][i]) return true;
  }
  return false;
}

static String getElectrodeSensorState(int bridgeIndex)
{
  if (electrodeStimActive[bridgeIndex]) return "STIMULATING";
  if (millis() < electrodeSilentUntil[bridgeIndex]) return "SILENT";
  if (electrodeSensorTriggerEnabled[bridgeIndex]) return "READY";
  return "DISABLED";
}

static String getTriggerModeLabel()
{
  switch (activeTriggerMode) {
    case TRIGGER_MODE_CYCLIC: return "CYCLIC";
    case TRIGGER_MODE_SENSOR_TRIGGERED: return "SENSOR_TRIGGERED";
    default: return "NONE";
  }
}

static bool setTriggerEventParam(const String& param, bool checked)
{
  int bridgeIndex = -1;
  int eventIndex = -1;

  if (param.startsWith("e1_")) bridgeIndex = 0;
  else if (param.startsWith("e2_")) bridgeIndex = 1;
  else return false;

  if (param.endsWith("s1_fall")) eventIndex = SENSOR1_FALLING;
  else if (param.endsWith("s2_fall")) eventIndex = SENSOR2_FALLING;
  else if (param.endsWith("s1_rise")) eventIndex = SENSOR1_RISING;
  else if (param.endsWith("s2_rise")) eventIndex = SENSOR2_RISING;
  else return false;

  electrodeTriggerEvents[bridgeIndex][eventIndex] = checked;

  portENTER_CRITICAL(&timerMux);
  if (electrodeSensorTriggerEnabled[bridgeIndex] && !anySelectedTriggerForElectrode(bridgeIndex)) {
    electrodeSensorTriggerEnabled[bridgeIndex] = false;
    electrodeCycleEnabled[bridgeIndex] = false;
    electrodeStimActive[bridgeIndex] = false;
    electrodeStimStartTime[bridgeIndex] = 0;
    electrodeSilentUntil[bridgeIndex] = 0;
    hBridges[bridgeIndex].enabled = false;
    hBridges[bridgeIndex].lastPulseState = -1;
    digitalWrite(hBridges[bridgeIndex].posPin, LOW);
    digitalWrite(hBridges[bridgeIndex].negPin, LOW);
  }
  sensorTriggerEnabled = electrodeSensorTriggerEnabled[0] || electrodeSensorTriggerEnabled[1];
  if (!sensorTriggerEnabled && activeTriggerMode == TRIGGER_MODE_SENSOR_TRIGGERED) {
    activeTriggerMode = TRIGGER_MODE_NONE;
    lastTriggerModePingTime = 0;
  }
  pulseOutputEnabled = electrodeStimActive[0] || electrodeStimActive[1] || (pulseCycleEnabled && cyclePhaseOn);
  portEXIT_CRITICAL(&timerMux);

  return true;
}

void handleSet() {
  if (server.hasArg("param") && server.hasArg("value")) {
    String param = server.arg("param");
    float value = server.arg("value").toFloat();

    if (setTriggerEventParam(param, value > 0.5f)) {
      server.send(200, "text/plain", "Sensor trigger event updated");
    }
    else if (param == "voltage") {
      if (value < 0.0f) value = 0.0f;
      TARGET_VOLTAGE = value;
      Serial.println("Set target voltage to " + String(value, 1) + "V");
      server.send(200, "text/plain", "Voltage set to " + String(value, 1) + "V");
    }
    else if (param == "kp") {
      if (value < 0.0f) value = 0.0f;
      Kp = value;
      Serial.println("Set boost controller Kp to " + String(Kp, 6));
      server.send(200, "text/plain", "Kp set to " + String(Kp, 6));
    }
    else if (param == "kd") {
      if (value < 0.0f) value = 0.0f;
      Kd = value;
      Serial.println("Set boost controller Kd to " + String(Kd, 6));
      server.send(200, "text/plain", "Kd set to " + String(Kd, 6));
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
    else if (param == "e1StimDuration") {
      if (value < 0.01f) value = 0.01f;
      electrodeStimDurationSeconds[0] = value;
      server.send(200, "text/plain", "Electrode 1 stimulation duration set to " + String(value, 2) + "s");
    }
    else if (param == "e2StimDuration") {
      if (value < 0.01f) value = 0.01f;
      electrodeStimDurationSeconds[1] = value;
      server.send(200, "text/plain", "Electrode 2 stimulation duration set to " + String(value, 2) + "s");
    }
    else if (param == "e1SilentPeriod") {
      if (value < 0.0f) value = 0.0f;
      electrodeSilentSeconds[0] = value;
      server.send(200, "text/plain", "Electrode 1 silent period set to " + String(value, 2) + "s");
    }
    else if (param == "e2SilentPeriod") {
      if (value < 0.0f) value = 0.0f;
      electrodeSilentSeconds[1] = value;
      server.send(200, "text/plain", "Electrode 2 silent period set to " + String(value, 2) + "s");
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
  json += "\"kp\":" + String(Kp, 6) + ",";
  json += "\"kd\":" + String(Kd, 6) + ",";
  json += "\"maxCurrent1\":" + String(hBridges[0].maxCurrent, 3) + ",";
  json += "\"maxCurrent2\":" + String(hBridges[1].maxCurrent, 3) + ",";
  json += "\"senseVoltage1\":" + String(senseVoltage1, 3) + ",";
  json += "\"senseVoltage2\":" + String(senseVoltage2, 3) + ",";
  json += "\"maxAllowedCurrent\":" + String(maxAllowedCurrent, 3) + ",";
  json += "\"overcurrentProtection1\":" + String(hBridges[0].overcurrentProtection ? "true" : "false") + ",";
  json += "\"timeSinceOvercurrent1\":" + String(timeSinceOvercurrent1) + ",";
  json += "\"overcurrentProtection2\":" + String(hBridges[1].overcurrentProtection ? "true" : "false") + ",";
  json += "\"timeSinceOvercurrent2\":" + String(timeSinceOvercurrent2) + ",";
  json += "\"triggerMode\":\"" + getTriggerModeLabel() + "\",";
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
  json += "\"electrode1SensorState\":\"" + getElectrodeSensorState(0) + "\",";
  json += "\"electrode2SensorState\":\"" + getElectrodeSensorState(1) + "\",";
  json += "\"electrode1StimDuration\":" + String(electrodeStimDurationSeconds[0], 2) + ",";
  json += "\"electrode2StimDuration\":" + String(electrodeStimDurationSeconds[1], 2) + ",";
  json += "\"electrode1SilentPeriod\":" + String(electrodeSilentSeconds[0], 2) + ",";
  json += "\"electrode2SilentPeriod\":" + String(electrodeSilentSeconds[1], 2) + ",";
  json += "\"e1Sensor1Falling\":" + String(electrodeTriggerEvents[0][SENSOR1_FALLING] ? "true" : "false") + ",";
  json += "\"e1Sensor2Falling\":" + String(electrodeTriggerEvents[0][SENSOR2_FALLING] ? "true" : "false") + ",";
  json += "\"e1Sensor1Rising\":" + String(electrodeTriggerEvents[0][SENSOR1_RISING] ? "true" : "false") + ",";
  json += "\"e1Sensor2Rising\":" + String(electrodeTriggerEvents[0][SENSOR2_RISING] ? "true" : "false") + ",";
  json += "\"e2Sensor1Falling\":" + String(electrodeTriggerEvents[1][SENSOR1_FALLING] ? "true" : "false") + ",";
  json += "\"e2Sensor2Falling\":" + String(electrodeTriggerEvents[1][SENSOR2_FALLING] ? "true" : "false") + ",";
  json += "\"e2Sensor1Rising\":" + String(electrodeTriggerEvents[1][SENSOR1_RISING] ? "true" : "false") + ",";
  json += "\"e2Sensor2Rising\":" + String(electrodeTriggerEvents[1][SENSOR2_RISING] ? "true" : "false") + ",";
  json += "\"pulseWidth1\":" + String(hBridges[0].pulseWidthUs) + ",";
  json += "\"pulseWidth2\":" + String(hBridges[1].pulseWidthUs);
  json += "}";

  server.send(200, "application/json", json);
}

void handleSensorLog()
{
  noteTriggerModePing();
  server.send(200, "application/json", consumeSensorEventLogJson());
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
  server.on("/sensor/log", handleSensorLog);
  server.on("/set", handleSet);
  server.on("/cycle/electrode1/on", handleCycleElectrode1On);
  server.on("/cycle/electrode2/on", handleCycleElectrode2On);
  server.on("/cycle/off", handleCycleOff);
  server.on("/sensor/electrode1/start", handleSensorTriggerElectrode1Start);
  server.on("/sensor/electrode2/start", handleSensorTriggerElectrode2Start);
  server.on("/sensor/off", handleSensorTriggerOff);
  
  server.begin();
  Serial.println("Web server started");
}

void updateWebServer()
{
  server.handleClient();
}

#include <WiFi.h>
#include <WebServer.h>

// WiFi credentials - UPDATE THESE WITH YOUR NETWORK
const char* ssid = "ESP32-AP";
const char* password = "123456789";

WebServer server(80);

// Hardware timer for pulse sequence
hw_timer_t *pulseTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

const int PIN_IN_PLUS   = 11;   // 1EDN7512 IN+
const int PIN_IN_MINUS  = 12;   // 1EDN7512 IN-
const int PIN_VSENSE    = 1;    // GPIO1 = ADC1_CH0

const int PIN_SENSOR_1  = 2;    // GPIO2 = ADC1_CH1
const int PIN_SENSOR_2  = 3;    // GPIO3 = ADC1_CH2

const int NUM_HBRIDGES = 2;

struct HBridgeChannel {
  const int posPin;
  const int negPin;
  const int comparatorPin;
  const int imaxPin;
  volatile int lastPulseState;
  volatile bool overcurrentProtection;
  volatile unsigned long overcurrentStartTime;
  volatile unsigned long pulseWidthUs;
  volatile bool enabled;
  float maxCurrent;
};

HBridgeChannel hBridges[NUM_HBRIDGES] = {
  {13, 14, 6, 16, -1, false, 0, 400, false, 0.5f},  // Electrode 1, I_MAX_1 = GPIO16
  {15, 17, 7, 18, -1, false, 0, 400, false, 0.5f},  // Electrode 2, I_MAX_2 = GPIO18
};

const uint32_t PWM_FREQ_HZ = 30000;
const uint8_t PWM_RES_BITS = 10;

// Divider resistors
const float R_TOP    = 1000000.0f; // 1 MΩ from Vout to ADC pin
const float R_BOTTOM = 10000.0f;    // 10 kΩ from ADC pin to GND

const float TARGET_DUTY = 0.20f;   // Final duty cycle = 3.0%

// Averaging variables for 1 second window
unsigned long avgStartTime = 0;

// =========================
// Feedback control settings
// =========================
const float TARGET_VOLTAGE_ON = 100.0f;    // Target voltage when pulse is ON
const float TARGET_VOLTAGE_OFF = 0.0f;     // Target voltage when pulse is OFF
float TARGET_VOLTAGE = TARGET_VOLTAGE_OFF; // Current target voltage (dynamic)
const float VOLTAGE_TOLERANCE = 0.10f;     // +/- tolerance in volts
const float FEEDBACK_STEP = 0.002f;        // Same style as your startup ramp step
const unsigned long FEEDBACK_INTERVAL_MS = 100;

// PD controller gains
const float Kp = 0.0001f;  // Proportional gain
const float Kd = 0.000f;  // Derivative gain

float currentDuty = 0.0f;
unsigned long lastFeedbackTime = 0;
float previousError = 0.0f;

// =========================
// H-bridge pulse settings
// =========================
const unsigned long PULSE_PERIOD_US = 25000;
const unsigned long PULSE_GAP_US = 100;
const unsigned long PULSE_WIDTH_MIN_US = 10;
const unsigned long PULSE_WIDTH_MAX_US = (PULSE_PERIOD_US - PULSE_GAP_US) / 2;

volatile unsigned long isrCallCount = 0;  // Debug counter for ISR calls

// Web control for pulse
volatile bool pulseManuallyEnabled = false;  // Controlled by web interface

// EMS cycle control (0.55s ON / 1.05s OFF)
bool pulseCycleEnabled = false;
bool electrodeCycleEnabled[NUM_HBRIDGES] = {false, false};
unsigned long cycleLastToggle = 0;
bool cyclePhaseOn = false;
const unsigned long CYCLE_ON_MS = 550;
const unsigned long CYCLE_OFF_MS = 1050;

// =========================
// Pressure sensor settings (Sensor 1 - GPIO2)
// =========================
float SENSOR1_THRESHOLD_PERCENT = 50.0f;   // Threshold in percentage (0-100)
const unsigned long PRESSURE_SAMPLE_MS = 10; // 100Hz = 10ms interval
float sensor1Percent = 0.0f;                 // Last sampled value as percentage (0-100)
bool pressureHigh = false;                   // true = above threshold
bool prevSensor1High = false;                // Previous state for edge detection
unsigned long lastPressureTime = 0;

// =========================
// Sensor 2 settings (GPIO3)
// =========================
float SENSOR2_THRESHOLD_PERCENT = 50.0f;   // Threshold in percentage (0-100)
float sensor2Percent = 0.0f;                 // Last sampled value as percentage (0-100)
bool sensor2High = false;                    // true = above threshold
bool prevSensor2High = false;                // Previous state for edge detection

// =========================
// Sensor-triggered state machine
// =========================
// Pulse turns ON at Sensor1 rising edge, OFF at Sensor2 falling edge
enum SmState { SM_WAITING, SM_ACTIVE };
SmState sensorSmState = SM_WAITING;
bool sensorTriggerEnabled = false;
bool electrodeSensorTriggerEnabled[NUM_HBRIDGES] = {false, false};
unsigned long sensorTriggerStartTime = 0;
float SENSOR_TRIGGER_MAX_STIM_SECONDS = 0.55f;

// =========================
// Overcurrent protection (per H-bridge)
// =========================
const unsigned long OVERCURRENT_RECOVERY_MS = 5000;
const int COMPARATOR_DEBOUNCE_CHECKS = 2;  // Number of consecutive LOW readings required
const int COMPARATOR_DEBOUNCE_DELAY_US = 2;  // Delay between debounce checks (microseconds)

// =========================
// Current limiting settings
// =========================
const float CURRENT_SENSE_RESISTOR = 16.5f;  // Ohms
const float ESP32_PWM_VOLTAGE = 3.3f;         // ESP32 PWM output voltage
const uint32_t IMAX_PWM_FREQ = 25000;         // 25kHz for ultra-smooth analog after RC filter (reduced noise)
const uint8_t IMAX_PWM_RES = 10;              // 10-bit resolution

float getMaxAllowedCurrent()
{
  return (0.95f * ESP32_PWM_VOLTAGE) / CURRENT_SENSE_RESISTOR;
}

void setDuty(float duty)
{
  if (duty < 0.0f) duty = 0.0f;
  if (duty > 0.95f) duty = 0.95f;

  currentDuty = duty;

  uint32_t maxDuty = (1U << PWM_RES_BITS) - 1U;
  uint32_t dutyCount = (uint32_t)(duty * maxDuty + 0.5f);
  ledcWrite(PIN_IN_PLUS, dutyCount);
}

void setMaxCurrent(int bridgeIndex, float current)
{
  if (bridgeIndex < 0 || bridgeIndex >= NUM_HBRIDGES) return;

  HBridgeChannel& hb = hBridges[bridgeIndex];
  float maxAllowedCurrent = getMaxAllowedCurrent();

  if (current < 0.0f) current = 0.0f;
  if (current > maxAllowedCurrent) {
    Serial.print("H-Bridge ");
    Serial.print(bridgeIndex + 1);
    Serial.print(" current limited from ");
    Serial.print(current, 3);
    Serial.print(" A to ");
    Serial.print(maxAllowedCurrent, 3);
    Serial.println(" A (95% duty cycle limit)");
    current = maxAllowedCurrent;
  }

  hb.maxCurrent = current;

  float targetVoltage = current * CURRENT_SENSE_RESISTOR;
  float duty = targetVoltage / ESP32_PWM_VOLTAGE;

  if (duty < 0.0f) duty = 0.0f;
  if (duty > 0.95f) duty = 0.95f;

  uint32_t maxDuty = (1U << IMAX_PWM_RES) - 1U;
  uint32_t dutyCount = (uint32_t)(duty * maxDuty + 0.5f);
  ledcWrite(hb.imaxPin, dutyCount);

  Serial.print("H-Bridge ");
  Serial.print(bridgeIndex + 1);
  Serial.print(" overcurrent limit set to ");
  Serial.print(current, 3);
  Serial.print(" A, Comparator ref: ");
  Serial.print(targetVoltage, 3);
  Serial.print(" V, IMAX PWM duty: ");
  Serial.print(duty * 100.0, 2);
  Serial.println(" %");
}

bool anyElectrodeSensorTriggerEnabled()
{
  for (int i = 0; i < NUM_HBRIDGES; i++) {
    if (electrodeSensorTriggerEnabled[i]) return true;
  }
  return false;
}

void setSensorTriggeredElectrodes(bool enabled)
{
  portENTER_CRITICAL(&timerMux);
  for (int i = 0; i < NUM_HBRIDGES; i++) {
    if (electrodeSensorTriggerEnabled[i]) {
      hBridges[i].enabled = enabled;
      if (!enabled) {
        digitalWrite(hBridges[i].posPin, LOW);
        digitalWrite(hBridges[i].negPin, LOW);
        hBridges[i].lastPulseState = -1;
      }
    }
  }
  pulseManuallyEnabled = enabled || hBridges[0].enabled || hBridges[1].enabled;
  portEXIT_CRITICAL(&timerMux);
}

void updatePressureSensor()
{
  if (millis() - lastPressureTime < PRESSURE_SAMPLE_MS) return;
  lastPressureTime = millis();

  // --- Sensor 1 (GPIO2) ---
  uint32_t adc1_raw = analogRead(PIN_SENSOR_1);
  sensor1Percent = (adc1_raw / 4095.0f) * 100.0f;
  bool sensor1HighNow = (sensor1Percent >= SENSOR1_THRESHOLD_PERCENT);

  // --- Sensor 2 (GPIO3) ---
  uint32_t adc2_raw = analogRead(PIN_SENSOR_2);
  sensor2Percent = (adc2_raw / 4095.0f) * 100.0f;  // Convert raw ADC to 0-100%
  bool sensor2HighNow = (sensor2Percent >= SENSOR2_THRESHOLD_PERCENT);

  // --- Sensor-triggered state machine ---
  if (sensorTriggerEnabled) {
    // Sensor 1 falling edge → turn pulses ON
    if (prevSensor1High && !sensor1HighNow && sensorSmState == SM_WAITING) {
      sensorSmState = SM_ACTIVE;
      sensorTriggerStartTime = millis();
      setSensorTriggeredElectrodes(true);
      Serial.println("SM: Sensor1 falling edge → selected electrodes ON");
    }
    // Sensor 2 rising edge → turn pulses OFF
    if (!prevSensor2High && sensor2HighNow && sensorSmState == SM_ACTIVE) {
      sensorSmState = SM_WAITING;
      sensorTriggerStartTime = 0;
      setSensorTriggeredElectrodes(false);
      Serial.println("SM: Sensor2 rising edge → selected electrodes OFF");
    }
    if (sensorSmState == SM_ACTIVE && sensorTriggerStartTime > 0) {
      unsigned long maxStimMs = (unsigned long)(SENSOR_TRIGGER_MAX_STIM_SECONDS * 1000.0f + 0.5f);
      if (millis() - sensorTriggerStartTime >= maxStimMs) {
        sensorSmState = SM_WAITING;
        sensorTriggerStartTime = 0;
        setSensorTriggeredElectrodes(false);
        Serial.println("SM: Maximum sensor trigger stimulation time reached → selected electrodes OFF");
      }
    }
  }

  // Update states
  pressureHigh  = sensor1HighNow;
  sensor2High   = sensor2HighNow;
  prevSensor1High = sensor1HighNow;
  prevSensor2High = sensor2HighNow;
}

float readOutputVoltage()
{
  uint32_t adc_mV = analogReadMilliVolts(PIN_VSENSE);
  float v_adc = adc_mV / 1000.0f;
  float v_out = v_adc * (R_TOP + R_BOTTOM) / R_BOTTOM;
  return v_out;
}


void updateFeedbackControl()
{
  if (millis() - lastFeedbackTime < FEEDBACK_INTERVAL_MS) {
    return;
  }
  
  unsigned long currentTime = millis();
  float dt = (currentTime - lastFeedbackTime) / 1000.0f; // Convert to seconds
  lastFeedbackTime = currentTime;

  float vout = readOutputVoltage();
  float error = TARGET_VOLTAGE - vout;
  
  // Calculate derivative term
  float derivative = (error - previousError) / dt;
  
  // PD control output
  float controlOutput = Kp * error + Kd * derivative;
  
  // Update duty cycle
  setDuty(currentDuty + controlOutput);
  
  // Store error for next iteration
  previousError = error;
  
  // Optional: print for debugging
  // Serial.print("Vout = ");
  // Serial.print(vout, 2);
  // Serial.print(" V, Error = ");
  // Serial.print(error, 2);
  // Serial.print(", Control = ");
  // Serial.println(controlOutput, 4);
}

void setHBridgeOff(HBridgeChannel& hb)
{
  digitalWrite(hb.posPin, LOW);
  digitalWrite(hb.negPin, LOW);
}

void setAllHBridgesOff()
{
  for (int i = 0; i < NUM_HBRIDGES; i++) {
    setHBridgeOff(hBridges[i]);
  }
}

void setPulseWidth(int bridgeIndex, unsigned long widthUs)
{
  if (bridgeIndex < 0 || bridgeIndex >= NUM_HBRIDGES) return;

  if (widthUs < PULSE_WIDTH_MIN_US) widthUs = PULSE_WIDTH_MIN_US;
  if (widthUs > PULSE_WIDTH_MAX_US) widthUs = PULSE_WIDTH_MAX_US;

  portENTER_CRITICAL(&timerMux);
  hBridges[bridgeIndex].pulseWidthUs = widthUs;
  portEXIT_CRITICAL(&timerMux);

  Serial.print("H-Bridge ");
  Serial.print(bridgeIndex + 1);
  Serial.print(" pulse width set to ");
  Serial.print(widthUs);
  Serial.println(" µs");
}

// ================================
// Comparator ISR for Overcurrent Detection
// ================================
void IRAM_ATTR onComparatorTriggerBridge(HBridgeChannel* hb) {
  int lowCount = 0;

  for (int i = 0; i < COMPARATOR_DEBOUNCE_CHECKS; i++) {
    if (digitalRead(hb->comparatorPin) == LOW) {
      lowCount++;
    }
    delayMicroseconds(COMPARATOR_DEBOUNCE_DELAY_US);
  }

  if (lowCount >= COMPARATOR_DEBOUNCE_CHECKS && !hb->overcurrentProtection) {
    hb->overcurrentProtection = true;
    hb->overcurrentStartTime = millis();
    digitalWrite(hb->posPin, LOW);
    digitalWrite(hb->negPin, LOW);
  }
}

void IRAM_ATTR onComparator0Trigger() {
  onComparatorTriggerBridge(&hBridges[0]);
}

void IRAM_ATTR onComparator1Trigger() {
  onComparatorTriggerBridge(&hBridges[1]);
}

void IRAM_ATTR updateHBridgePulse(HBridgeChannel& hb, unsigned long phaseUs, bool enabled) {
  if (!enabled || hb.overcurrentProtection) {
    if (hb.lastPulseState != -1) {
      digitalWrite(hb.posPin, LOW);
      digitalWrite(hb.negPin, LOW);
      hb.lastPulseState = -1;
    }
    return;
  }

  unsigned long width = hb.pulseWidthUs;
  unsigned long gapEnd = width + PULSE_GAP_US;
  unsigned long negEnd = gapEnd + width;

  if (phaseUs < width) {
    if (hb.lastPulseState != 0) {
      digitalWrite(hb.negPin, LOW);
      digitalWrite(hb.posPin, HIGH);
      hb.lastPulseState = 0;
    }
  }
  else if (phaseUs < gapEnd) {
    if (hb.lastPulseState != 1) {
      digitalWrite(hb.posPin, LOW);
      digitalWrite(hb.negPin, LOW);
      hb.lastPulseState = 1;
    }
  }
  else if (phaseUs < negEnd) {
    if (hb.lastPulseState != 2) {
      digitalWrite(hb.posPin, LOW);
      digitalWrite(hb.negPin, HIGH);
      hb.lastPulseState = 2;
    }
  }
  else {
    if (hb.lastPulseState != 3) {
      digitalWrite(hb.posPin, LOW);
      digitalWrite(hb.negPin, LOW);
      hb.lastPulseState = 3;
    }
  }
}

// ================================
// Hardware Timer ISR for H-bridge pulse sequence
// ================================
void IRAM_ATTR onPulseTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  isrCallCount++;

  unsigned long phaseUs = micros() % PULSE_PERIOD_US;
  bool enabled = pulseManuallyEnabled;

  for (int i = 0; i < NUM_HBRIDGES; i++) {
    updateHBridgePulse(hBridges[i], phaseUs, enabled && hBridges[i].enabled);
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}


// ================================
// Web Server Handlers
// ================================

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
  html += "      var pColor = data.pressureHigh ? 'green' : 'orange';";
  html += "      document.querySelectorAll('.status')[11].innerHTML = 'Sensor 1 (GPIO2): <span style=\"color:' + pColor + ';font-weight:bold;\">' + (data.pressureHigh ? 'HIGH' : 'LOW') + '</span> (' + data.sensor1Percent.toFixed(1) + '%)';";
  html += "      var s2Color = data.sensor2High ? 'green' : 'orange';";
  html += "      document.querySelectorAll('.status')[12].innerHTML = 'Sensor 2 (GPIO3): <span style=\"color:' + s2Color + ';font-weight:bold;\">' + (data.sensor2High ? 'HIGH' : 'LOW') + '</span> (' + data.sensor2Percent.toFixed(1) + '%)';";
  html += "      var smColor = data.sensorTriggerEnabled ? (data.smActive ? 'green' : 'blue') : 'gray';";
  html += "      document.querySelectorAll('.status')[13].innerHTML = 'Sensor Trigger SM: <span style=\"color:' + smColor + ';font-weight:bold;\">' + (data.sensorTriggerEnabled ? (data.smActive ? 'ACTIVE' : 'WAITING') : 'DISABLED') + '</span> | Max Time: <span class=\"value\">' + data.sensorTriggerMaxStimSeconds.toFixed(2) + ' s</span> | Electrode 1 Trigger: <span class=\"value\">' + (data.electrode1SensorTriggerEnabled ? 'ON' : 'OFF') + '</span> | Electrode 2 Trigger: <span class=\"value\">' + (data.electrode2SensorTriggerEnabled ? 'ON' : 'OFF') + '</span>';";
  html += "      document.querySelectorAll('.status')[14].innerHTML = 'Electrode 1 Pulse Width: <span class=\"value\">' + data.pulseWidth1 + ' µs</span>';";
  html += "      document.querySelectorAll('.status')[15].innerHTML = 'Electrode 2 Pulse Width: <span class=\"value\">' + data.pulseWidth2 + ' µs</span>';";
  html += "    });";  
  html += "}";
  html += "setInterval(updateStatus, 1000);";
  html += "</script>";
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h1>ESP32 FES Control Panel</h1>";
  
  // Electrode control (at the top)
  html += "<div class='section'>";
  html += "<h2>Electrode Control</h2>";
  html += "<h3 style='margin: 10px 0;'>Electrode 1</h3>";
  html += "<button onclick=\"location.href='/hbridge1/on'\" style='background: #4CAF50;'>Turn Electrode 1 ON</button>";
  html += "<button onclick=\"location.href='/hbridge1/off'\" class='btn-off'>Turn Electrode 1 OFF</button>";
  html += "<h3 style='margin: 10px 0;'>Electrode 2</h3>";
  html += "<button onclick=\"location.href='/hbridge2/on'\" style='background: #4CAF50;'>Turn Electrode 2 ON</button>";
  html += "<button onclick=\"location.href='/hbridge2/off'\" class='btn-off'>Turn Electrode 2 OFF</button>";
  html += "<hr style='margin: 15px 0;'>";
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
  
  // Status section
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
  String pColor = pressureHigh ? "green" : "orange";
  html += "<div class='status'>Sensor 1 (GPIO2): <span style='color:" + pColor + ";font-weight:bold;'>" + String(pressureHigh ? "HIGH" : "LOW") + "</span> (" + String(sensor1Percent, 1) + "%)</div>";
  String s2Color = sensor2High ? "green" : "orange";
  html += "<div class='status'>Sensor 2 (GPIO3): <span style='color:" + s2Color + ";font-weight:bold;'>" + String(sensor2High ? "HIGH" : "LOW") + "</span> (" + String(sensor2Percent, 1) + "%)</div>";
  String smColor = sensorTriggerEnabled ? (sensorSmState == SM_ACTIVE ? "green" : "blue") : "gray";
  String smLabel = sensorTriggerEnabled ? (sensorSmState == SM_ACTIVE ? "ACTIVE" : "WAITING") : "DISABLED";
  html += "<div class='status'>Sensor Trigger SM: <span style='color:" + smColor + ";font-weight:bold;'>" + smLabel + "</span> | Max Time: <span class='value'>" + String(SENSOR_TRIGGER_MAX_STIM_SECONDS, 2) + " s</span> | Electrode 1 Trigger: <span class='value'>" + String(electrodeSensorTriggerEnabled[0] ? "ON" : "OFF") + "</span> | Electrode 2 Trigger: <span class='value'>" + String(electrodeSensorTriggerEnabled[1] ? "ON" : "OFF") + "</span></div>";
  html += "<div class='status'>Electrode 1 Pulse Width: <span class='value'>" + String(hBridges[0].pulseWidthUs) + " µs</span></div>";
  html += "<div class='status'>Electrode 2 Pulse Width: <span class='value'>" + String(hBridges[1].pulseWidthUs) + " µs</span></div>";
  html += "</div>";
  
  // Voltage control
  html += "<div class='section'>";
  html += "<h2>Voltage Control</h2>";
  html += "<div class='label'>Target Voltage (V):</div>";
  html += "<input type='number' id='voltage' value='" + String(TARGET_VOLTAGE, 1) + "' step='1' min='0'>";
  html += "<button onclick=\"adjustVoltage(2.5)\" style='background: #2196F3;'>+2.5 V</button>";
  html += "<button onclick=\"adjustVoltage(-2.5)\" style='background: #607D8B;'>-2.5 V</button>";
  html += "<button onclick=\"updateValue('voltage', document.getElementById('voltage').value)\">Set Voltage</button>";
  html += "</div>";
  
  // Overcurrent limit control (per H-bridge)
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
  html += "Current sense resistor: " + String(CURRENT_SENSE_RESISTOR, 1) + "Ω<br>";
  html += "Electrode 1 ref: " + String(senseVoltage1, 3) + "V, Electrode 2 ref: " + String(senseVoltage2, 3) + "V<br>";
  html += "<strong>Max allowed (95% duty): " + String(maxAllowedCurrent, 3) + "A per channel</strong>";
  html += "</div>";
  html += "</div>";

  // Pulse width control (per H-bridge)
  html += "<div class='section'>";
  html += "<h2>Pulse Width Control</h2>";
  html += "<div class='label'>Electrode 1 Pulse Width (µs): <span class='value'>" + String(hBridges[0].pulseWidthUs) + "</span></div>";
  html += "<input type='number' id='width1' value='" + String(hBridges[0].pulseWidthUs) + "' step='10' min='" + String(PULSE_WIDTH_MIN_US) + "' max='" + String(PULSE_WIDTH_MAX_US) + "'>";
  html += "<button onclick=\"updateValue('width1', document.getElementById('width1').value)\">Set Electrode 1 Width</button>";
  html += "<div class='label'>Electrode 2 Pulse Width (µs): <span class='value'>" + String(hBridges[1].pulseWidthUs) + "</span></div>";
  html += "<input type='number' id='width2' value='" + String(hBridges[1].pulseWidthUs) + "' step='10' min='" + String(PULSE_WIDTH_MIN_US) + "' max='" + String(PULSE_WIDTH_MAX_US) + "'>";
  html += "<button onclick=\"updateValue('width2', document.getElementById('width2').value)\">Set Electrode 2 Width</button>";
  html += "<div style='margin-top: 10px; font-size: 0.9em; color: #666;'>";
  html += "Period: " + String(PULSE_PERIOD_US) + " µs, Gap: " + String(PULSE_GAP_US) + " µs<br>";
  html += "Max width per bridge: " + String(PULSE_WIDTH_MAX_US) + " µs";
  html += "</div>";
  html += "</div>";

  // Sensor threshold control
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

void handlePulseOn() {
  pulseManuallyEnabled = true;
  Serial.println("Web: Pulses turned ON");
  server.sendHeader("Location", "/");
  server.send(303);
}

void handlePulseOff() {
  pulseManuallyEnabled = false;
  Serial.println("Web: Pulses turned OFF");
  server.sendHeader("Location", "/");
  server.send(303);
}

void setHBridgeEnabled(int bridgeIndex, bool enabled) {
  if (bridgeIndex < 0 || bridgeIndex >= NUM_HBRIDGES) return;

  portENTER_CRITICAL(&timerMux);
  hBridges[bridgeIndex].enabled = enabled;
  if (enabled) {
    pulseManuallyEnabled = true;
  }
  if (!enabled) {
    electrodeCycleEnabled[bridgeIndex] = false;
    electrodeSensorTriggerEnabled[bridgeIndex] = false;
    digitalWrite(hBridges[bridgeIndex].posPin, LOW);
    digitalWrite(hBridges[bridgeIndex].negPin, LOW);
    hBridges[bridgeIndex].lastPulseState = -1;
    bool anyCycleEnabled = false;
    for (int i = 0; i < NUM_HBRIDGES; i++) {
      if (electrodeCycleEnabled[i]) {
        anyCycleEnabled = true;
        break;
      }
    }
    if (!anyCycleEnabled) {
      pulseCycleEnabled = false;
      cyclePhaseOn = false;
    }
    if (!anyElectrodeSensorTriggerEnabled()) {
      sensorTriggerEnabled = false;
      sensorSmState = SM_WAITING;
      sensorTriggerStartTime = 0;
    }
    pulseManuallyEnabled = hBridges[0].enabled || hBridges[1].enabled;
  }
  portEXIT_CRITICAL(&timerMux);

  Serial.print("Web: Electrode ");
  Serial.print(bridgeIndex + 1);
  Serial.println(enabled ? " enabled" : " disabled");
}

void handleHBridge1On() {
  setHBridgeEnabled(0, true);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleHBridge1Off() {
  setHBridgeEnabled(0, false);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleHBridge2On() {
  setHBridgeEnabled(1, true);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleHBridge2Off() {
  setHBridgeEnabled(1, false);
  server.sendHeader("Location", "/");
  server.send(303);
}

bool anyElectrodeCycleEnabled() {
  for (int i = 0; i < NUM_HBRIDGES; i++) {
    if (electrodeCycleEnabled[i]) return true;
  }
  return false;
}

void startElectrodeCycle(int bridgeIndex) {
  if (bridgeIndex < 0 || bridgeIndex >= NUM_HBRIDGES) return;

  portENTER_CRITICAL(&timerMux);
  electrodeCycleEnabled[bridgeIndex] = true;
  hBridges[bridgeIndex].enabled = true;
  pulseCycleEnabled = true;
  cyclePhaseOn = true;
  cycleLastToggle = millis();
  pulseManuallyEnabled = true;
  portEXIT_CRITICAL(&timerMux);

  Serial.print("Web: Electrode ");
  Serial.print(bridgeIndex + 1);
  Serial.println(" EMS cycle started (0.55s ON / 1.05s OFF)");
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
  pulseManuallyEnabled = false;
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

void updatePulseCycle() {
  if (!pulseCycleEnabled) return;
  if (!anyElectrodeCycleEnabled()) {
    pulseCycleEnabled = false;
    pulseManuallyEnabled = false;
    return;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - cycleLastToggle;

  if (cyclePhaseOn && elapsed >= CYCLE_ON_MS) {
    pulseManuallyEnabled = false;
    cyclePhaseOn = false;
    cycleLastToggle = now;
    Serial.println("EMS Cycle: OFF phase");
  } else if (!cyclePhaseOn && elapsed >= CYCLE_OFF_MS) {
    pulseManuallyEnabled = true;
    cyclePhaseOn = true;
    cycleLastToggle = now;
    Serial.println("EMS Cycle: ON phase");
  }
}

void startElectrodeSensorTrigger(int bridgeIndex) {
  if (bridgeIndex < 0 || bridgeIndex >= NUM_HBRIDGES) return;

  portENTER_CRITICAL(&timerMux);
  electrodeSensorTriggerEnabled[bridgeIndex] = true;
  electrodeCycleEnabled[bridgeIndex] = false;
  hBridges[bridgeIndex].enabled = false;
  digitalWrite(hBridges[bridgeIndex].posPin, LOW);
  digitalWrite(hBridges[bridgeIndex].negPin, LOW);
  hBridges[bridgeIndex].lastPulseState = -1;
  sensorTriggerEnabled = true;
  sensorSmState = SM_WAITING;
  sensorTriggerStartTime = 0;
  pulseManuallyEnabled = hBridges[0].enabled || hBridges[1].enabled;
  if (!anyElectrodeCycleEnabled()) {
    pulseCycleEnabled = false;
    cyclePhaseOn = false;
  }
  portEXIT_CRITICAL(&timerMux);

  Serial.print("Web: Electrode ");
  Serial.print(bridgeIndex + 1);
  Serial.println(" sensor trigger ENABLED");
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
  pulseManuallyEnabled = hBridges[0].enabled || hBridges[1].enabled;
  portEXIT_CRITICAL(&timerMux);

  Serial.println("Web: Sensor trigger mode DISABLED");
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSetVoltage() {
  if (server.hasArg("voltage")) {
    float newVoltage = server.arg("voltage").toFloat();
    
    // Validate voltage range (minimum 0V)
    if (newVoltage < 0.0f) newVoltage = 0.0f;
    
    TARGET_VOLTAGE = newVoltage;
    
    Serial.println("Web: Target voltage set to " + String(newVoltage, 1) + "V");
  }
  
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
      server.send(200, "text/plain", "Electrode 1 pulse width set to " + String(hBridges[0].pulseWidthUs) + " µs");
    }
    else if (param == "width2") {
      setPulseWidth(1, (unsigned long)value);
      server.send(200, "text/plain", "Electrode 2 pulse width set to " + String(hBridges[1].pulseWidthUs) + " µs");
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
  json += "\"pulseEnabled\":" + String(pulseManuallyEnabled ? "true" : "false") + ",";
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
  json += "\"pressureHigh\":" + String(pressureHigh ? "true" : "false") + ",";
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

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-S3 Boost Driver + Dual H-bridge Pulse");

  // Initialize WiFi as Access Point
  Serial.println("Setting up WiFi Access Point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.println("Connect to the AP and navigate to: http://" + IP.toString());

  // Configure web server routes
  server.on("/", handleRoot);
  server.on("/pulse/on", handlePulseOn);
  server.on("/pulse/off", handlePulseOff);
  server.on("/hbridge1/on", handleHBridge1On);
  server.on("/hbridge1/off", handleHBridge1Off);
  server.on("/hbridge2/on", handleHBridge2On);
  server.on("/hbridge2/off", handleHBridge2Off);
  server.on("/setVoltage", HTTP_POST, handleSetVoltage);
  server.on("/status", handleStatus);
  server.on("/set", handleSet);
  server.on("/cycle/electrode1/on", handleCycleElectrode1On);
  server.on("/cycle/electrode2/on", handleCycleElectrode2On);
  server.on("/cycle/off", handleCycleOff);
  server.on("/sensor/on", handleSensorTriggerOn);
  server.on("/sensor/electrode1/on", handleSensorTriggerElectrode1On);
  server.on("/sensor/electrode2/on", handleSensorTriggerElectrode2On);
  server.on("/sensor/off", handleSensorTriggerOff);
  
  // Start web server
  server.begin();
  Serial.println("Web server started");

  pinMode(PIN_IN_MINUS, OUTPUT);
  digitalWrite(PIN_IN_MINUS, LOW);

  for (int i = 0; i < NUM_HBRIDGES; i++) {
    pinMode(hBridges[i].posPin, OUTPUT);
    pinMode(hBridges[i].negPin, OUTPUT);
    pinMode(hBridges[i].comparatorPin, INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(hBridges[0].comparatorPin), onComparator0Trigger, FALLING);
  attachInterrupt(digitalPinToInterrupt(hBridges[1].comparatorPin), onComparator1Trigger, FALLING);
  Serial.println("Overcurrent protection enabled on GPIO6 (H-Bridge 1) and GPIO7 (H-Bridge 2)");

  // Setup PWM for IMAX pins (comparator reference voltage per H-bridge)
  for (int i = 0; i < NUM_HBRIDGES; i++) {
    if (!ledcAttach(hBridges[i].imaxPin, IMAX_PWM_FREQ, IMAX_PWM_RES)) {
      Serial.print("IMAX PWM attach failed on GPIO");
      Serial.println(hBridges[i].imaxPin);
      while (true) {
        delay(1000);
      }
    }
    setMaxCurrent(i, hBridges[i].maxCurrent);
  }

  setAllHBridgesOff();

  // ADC setup
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VSENSE, ADC_11db);
  analogSetPinAttenuation(PIN_SENSOR_1, ADC_11db);
  analogSetPinAttenuation(PIN_SENSOR_2, ADC_11db);

  // PWM setup for boost switch
  if (!ledcAttach(PIN_IN_PLUS, PWM_FREQ_HZ, PWM_RES_BITS)) {
    Serial.println("LEDC attach failed");
    while (true) {
      delay(1000);
    }
  }

  // Start from 0%
  setDuty(0.0f);
  delay(1000);

  // Initialize hardware timer for H-bridge pulse sequence
  // Create timer at 20kHz tick rate
  pulseTimer = timerBegin(20000);  // 20kHz frequency
  if (pulseTimer == NULL) {
    Serial.println("ERROR: Timer initialization failed!");
  } else {
    Serial.println("Timer created successfully at 20kHz");
  }
  
  // Attach the ISR to the timer
  timerAttachInterrupt(pulseTimer, &onPulseTimer);
  Serial.println("ISR attached to timer");
  
  // Set alarm to trigger every 1 tick (gives us 20kHz interrupt rate)
  // timerAlarm(timer, alarm_value, autoreload, reload_count)
  // alarm_value = 1 means interrupt every tick
  // autoreload = true means repeat forever
  // reload_count = 0 means unlimited
  timerAlarm(pulseTimer, 1, true, 0);
  Serial.println("Timer alarm set to trigger every tick (20kHz rate)");
  
  Serial.println("Hardware timer initialized for dual H-bridge pulse control");

  avgStartTime = millis();
  lastFeedbackTime = millis();
  lastPressureTime = millis();

  Serial.println("System ready. Dual H-bridge pulse running on hardware timer.");
  Serial.println("Use web interface to control target voltage (pulse ON/OFF).");
}

void loop()
{
  // Handle web server requests
  server.handleClient();

  // Closed-loop voltage control
  updateFeedbackControl();

  // EMS cycle mode (0.55s ON / 1.05s OFF)
  updatePulseCycle();

  // Pressure sensor sampling at 100Hz
  updatePressureSensor();

  // Per-bridge overcurrent recovery
  for (int i = 0; i < NUM_HBRIDGES; i++) {
    if (hBridges[i].overcurrentProtection) {
      unsigned long timeSinceOvercurrent = millis() - hBridges[i].overcurrentStartTime;

      if (timeSinceOvercurrent >= OVERCURRENT_RECOVERY_MS) {
        hBridges[i].overcurrentProtection = false;
        Serial.print("H-Bridge ");
        Serial.print(i + 1);
        Serial.print(" overcurrent recovery attempt after ");
        Serial.print(timeSinceOvercurrent);
        Serial.println(" ms");
      }
    }
  }

  // Every 1 second, print diagnostic info
  if (millis() - avgStartTime >= 1000) {
    float vout = readOutputVoltage();
    Serial.print("ISR calls in last second: ");
    Serial.print(isrCallCount);
    Serial.print(" | HB1 state: ");
    Serial.print(hBridges[0].lastPulseState);
    Serial.print(" | HB2 state: ");
    Serial.print(hBridges[1].lastPulseState);
    Serial.print(" | Target V: ");
    Serial.print(TARGET_VOLTAGE, 1);
    Serial.print(" | Current V: ");
    Serial.print(vout, 1);
    Serial.print(" | OC1: ");
    Serial.print(hBridges[0].overcurrentProtection ? "YES" : "NO");
    Serial.print(" | OC2: ");
    Serial.println(hBridges[1].overcurrentProtection ? "YES" : "NO");

    isrCallCount = 0;  // Reset counter
    avgStartTime = millis();
  }
}

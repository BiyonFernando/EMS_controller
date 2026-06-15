#include "FesController.h"

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
  Serial.println(" us");
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

void IRAM_ATTR onPulseTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  unsigned long phaseUs = micros() % PULSE_PERIOD_US;
  bool cycleActive = pulseCycleEnabled && cyclePhaseOn;

  for (int i = 0; i < NUM_HBRIDGES; i++) {
    bool bridgeEnabled = electrodeStimActive[i] || (electrodeCycleEnabled[i] && cycleActive);
    updateHBridgePulse(hBridges[i], phaseUs, bridgeEnabled && hBridges[i].enabled);
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}

bool anyElectrodeCycleEnabled() {
  for (int i = 0; i < NUM_HBRIDGES; i++) {
    if (electrodeCycleEnabled[i]) return true;
  }
  return false;
}

static bool anySensorStimActiveForPulseGate()
{
  for (int i = 0; i < NUM_HBRIDGES; i++) {
    if (electrodeStimActive[i]) return true;
  }
  return false;
}

void startElectrodeCycle(int bridgeIndex) {
  if (bridgeIndex < 0 || bridgeIndex >= NUM_HBRIDGES) return;

  portENTER_CRITICAL(&timerMux);
  electrodeSensorTriggerEnabled[bridgeIndex] = false;
  electrodeStimActive[bridgeIndex] = false;
  electrodeStimStartTime[bridgeIndex] = 0;
  electrodeSilentUntil[bridgeIndex] = 0;
  for (int eventIndex = 0; eventIndex < SENSOR_TRIGGER_EVENT_COUNT; eventIndex++) {
    electrodeTriggerEvents[bridgeIndex][eventIndex] = false;
  }
  sensorTriggerEnabled = electrodeSensorTriggerEnabled[0] || electrodeSensorTriggerEnabled[1];
  electrodeCycleEnabled[bridgeIndex] = true;
  hBridges[bridgeIndex].enabled = true;
  pulseCycleEnabled = true;
  cyclePhaseOn = true;
  cycleLastToggle = millis();
  pulseOutputEnabled = true;
  portEXIT_CRITICAL(&timerMux);

  Serial.print("Web: Electrode ");
  Serial.print(bridgeIndex + 1);
  Serial.println(" EMS cycle started (0.55s ON / 1.05s OFF)");
}

void updatePulseCycle() {
  if (!pulseCycleEnabled) return;
  if (!anyElectrodeCycleEnabled()) {
    pulseCycleEnabled = false;
    pulseOutputEnabled = anySensorStimActiveForPulseGate();
    return;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - cycleLastToggle;

  if (cyclePhaseOn && elapsed >= CYCLE_ON_MS) {
    pulseOutputEnabled = anySensorStimActiveForPulseGate();
    cyclePhaseOn = false;
    cycleLastToggle = now;
    Serial.println("EMS Cycle: OFF phase");
  } else if (!cyclePhaseOn && elapsed >= CYCLE_OFF_MS) {
    pulseOutputEnabled = true;
    cyclePhaseOn = true;
    cycleLastToggle = now;
    Serial.println("EMS Cycle: ON phase");
  }
}

void setupPulseOutputs()
{
  pinMode(PIN_IN_MINUS, OUTPUT);
  digitalWrite(PIN_IN_MINUS, LOW);

  for (int i = 0; i < NUM_HBRIDGES; i++) {
    pinMode(hBridges[i].posPin, OUTPUT);
    pinMode(hBridges[i].negPin, OUTPUT);
    pinMode(hBridges[i].comparatorPin, INPUT_PULLUP);
  }
}

void setupPulseTimer()
{
  pulseTimer = timerBegin(20000);
  if (pulseTimer == NULL) {
    Serial.println("ERROR: Timer initialization failed!");
  } else {
    Serial.println("Timer created successfully at 20kHz");
  }
  
  timerAttachInterrupt(pulseTimer, &onPulseTimer);
  Serial.println("ISR attached to timer");

  timerAlarm(pulseTimer, 1, true, 0);
  Serial.println("Timer alarm set to trigger every tick (20kHz rate)");
  Serial.println("Hardware timer initialized for dual H-bridge pulse control");
}

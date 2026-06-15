#include "FesController.h"

static unsigned long secondsToMillis(float seconds)
{
  if (seconds < 0.0f) seconds = 0.0f;
  return (unsigned long)(seconds * 1000.0f + 0.5f);
}

static bool anySensorTriggerElectrodeEnabled()
{
  for (int i = 0; i < NUM_HBRIDGES; i++) {
    if (electrodeSensorTriggerEnabled[i]) return true;
  }
  return false;
}

static bool anyTriggerEventSelected(int bridgeIndex)
{
  for (int i = 0; i < SENSOR_TRIGGER_EVENT_COUNT; i++) {
    if (electrodeTriggerEvents[bridgeIndex][i]) return true;
  }
  return false;
}

static bool anySensorStimActive()
{
  for (int i = 0; i < NUM_HBRIDGES; i++) {
    if (electrodeStimActive[i]) return true;
  }
  return false;
}

static bool triggerSelected(int bridgeIndex, const bool events[SENSOR_TRIGGER_EVENT_COUNT])
{
  if (!electrodeSensorTriggerEnabled[bridgeIndex]) return false;
  if (electrodeStimActive[bridgeIndex]) return false;
  if (millis() < electrodeSilentUntil[bridgeIndex]) return false;

  for (int i = 0; i < SENSOR_TRIGGER_EVENT_COUNT; i++) {
    if (events[i] && electrodeTriggerEvents[bridgeIndex][i]) return true;
  }
  return false;
}

static void setSensorTriggeredElectrode(int bridgeIndex, bool enabled)
{
  if (bridgeIndex < 0 || bridgeIndex >= NUM_HBRIDGES) return;

  portENTER_CRITICAL(&timerMux);
  hBridges[bridgeIndex].enabled = enabled;
  if (!enabled) {
    digitalWrite(hBridges[bridgeIndex].posPin, LOW);
    digitalWrite(hBridges[bridgeIndex].negPin, LOW);
    hBridges[bridgeIndex].lastPulseState = -1;
  }
  pulseOutputEnabled = anySensorStimActive() || (pulseCycleEnabled && cyclePhaseOn);
  portEXIT_CRITICAL(&timerMux);
}

static void startSensorStim(int bridgeIndex)
{
  electrodeStimActive[bridgeIndex] = true;
  electrodeStimStartTime[bridgeIndex] = millis();
  setSensorTriggeredElectrode(bridgeIndex, true);

  Serial.print("Sensor trigger: Electrode ");
  Serial.print(bridgeIndex + 1);
  Serial.println(" ON");
}

static void stopSensorStim(int bridgeIndex)
{
  electrodeStimActive[bridgeIndex] = false;
  electrodeStimStartTime[bridgeIndex] = 0;
  electrodeSilentUntil[bridgeIndex] = millis() + secondsToMillis(electrodeSilentSeconds[bridgeIndex]);
  setSensorTriggeredElectrode(bridgeIndex, false);

  Serial.print("Sensor trigger: Electrode ");
  Serial.print(bridgeIndex + 1);
  Serial.println(" OFF, silent period started");
}

static void updateSensorStimTimers()
{
  for (int i = 0; i < NUM_HBRIDGES; i++) {
    if (!electrodeStimActive[i]) continue;

    unsigned long durationMs = secondsToMillis(electrodeStimDurationSeconds[i]);
    if (millis() - electrodeStimStartTime[i] >= durationMs) {
      stopSensorStim(i);
    }
  }
}

static void handleSensorTriggerEvents(bool sensor1HighNow, bool sensor2HighNow)
{
  bool events[SENSOR_TRIGGER_EVENT_COUNT] = {
    prevSensor1High && !sensor1HighNow,
    prevSensor2High && !sensor2HighNow,
    !prevSensor1High && sensor1HighNow,
    !prevSensor2High && sensor2HighNow,
  };

  for (int i = 0; i < NUM_HBRIDGES; i++) {
    if (triggerSelected(i, events)) {
      startSensorStim(i);
    }
  }
}

void updatePressureSensor()
{
  updateSensorStimTimers();

  if (millis() - lastPressureTime < PRESSURE_SAMPLE_MS) return;
  lastPressureTime = millis();

  uint32_t adc1_raw = analogRead(PIN_SENSOR_1);
  sensor1Percent = (adc1_raw / 4095.0f) * 100.0f;
  bool sensor1HighNow = (sensor1Percent >= SENSOR1_THRESHOLD_PERCENT);

  uint32_t adc2_raw = analogRead(PIN_SENSOR_2);
  sensor2Percent = (adc2_raw / 4095.0f) * 100.0f;
  bool sensor2HighNow = (sensor2Percent >= SENSOR2_THRESHOLD_PERCENT);

  if (sensorTriggerEnabled) {
    handleSensorTriggerEvents(sensor1HighNow, sensor2HighNow);
  }

  sensor1High = sensor1HighNow;
  sensor2High = sensor2HighNow;
  prevSensor1High = sensor1HighNow;
  prevSensor2High = sensor2HighNow;
}

void startElectrodeSensorTrigger(int bridgeIndex)
{
  if (bridgeIndex < 0 || bridgeIndex >= NUM_HBRIDGES) return;

  if (!anyTriggerEventSelected(bridgeIndex)) {
    Serial.print("Web: Electrode ");
    Serial.print(bridgeIndex + 1);
    Serial.println(" sensor trigger not started because no trigger events are selected");
    return;
  }

  portENTER_CRITICAL(&timerMux);
  electrodeSensorTriggerEnabled[bridgeIndex] = true;
  electrodeCycleEnabled[bridgeIndex] = false;
  electrodeStimActive[bridgeIndex] = false;
  electrodeStimStartTime[bridgeIndex] = 0;
  electrodeSilentUntil[bridgeIndex] = 0;
  hBridges[bridgeIndex].enabled = false;
  hBridges[bridgeIndex].lastPulseState = -1;
  digitalWrite(hBridges[bridgeIndex].posPin, LOW);
  digitalWrite(hBridges[bridgeIndex].negPin, LOW);
  sensorTriggerEnabled = true;
  if (!anyElectrodeCycleEnabled()) {
    pulseCycleEnabled = false;
    cyclePhaseOn = false;
  }
  pulseOutputEnabled = anySensorStimActive() || (pulseCycleEnabled && cyclePhaseOn);
  portEXIT_CRITICAL(&timerMux);

  Serial.print("Web: Electrode ");
  Serial.print(bridgeIndex + 1);
  Serial.println(" sensor trigger STARTED");
}

void stopAllSensorTriggers()
{
  portENTER_CRITICAL(&timerMux);
  for (int i = 0; i < NUM_HBRIDGES; i++) {
    bool wasSensorControlled = electrodeSensorTriggerEnabled[i] || electrodeStimActive[i];
    electrodeSensorTriggerEnabled[i] = false;
    electrodeStimActive[i] = false;
    electrodeStimStartTime[i] = 0;
    electrodeSilentUntil[i] = 0;
    for (int eventIndex = 0; eventIndex < SENSOR_TRIGGER_EVENT_COUNT; eventIndex++) {
      electrodeTriggerEvents[i][eventIndex] = false;
    }
    if (wasSensorControlled && !electrodeCycleEnabled[i]) {
      hBridges[i].enabled = false;
      hBridges[i].lastPulseState = -1;
      digitalWrite(hBridges[i].posPin, LOW);
      digitalWrite(hBridges[i].negPin, LOW);
    }
  }
  sensorTriggerEnabled = false;
  pulseOutputEnabled = pulseCycleEnabled && cyclePhaseOn;
  portEXIT_CRITICAL(&timerMux);

  Serial.println("Web: Sensor trigger mode DISABLED");
}

void setupSensorControl()
{
  lastPressureTime = millis();
  sensorTriggerEnabled = anySensorTriggerElectrodeEnabled();
}

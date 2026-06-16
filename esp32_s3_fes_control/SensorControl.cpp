#include "FesController.h"
#include <string.h>

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

static bool triggerSelectedForEvent(int bridgeIndex, SensorTriggerEvent event)
{
  if (!electrodeSensorTriggerEnabled[bridgeIndex]) return false;
  if (electrodeStimActive[bridgeIndex]) return false;
  if (millis() < electrodeSilentUntil[bridgeIndex]) return false;
  return electrodeTriggerEvents[bridgeIndex][event];
}

static const char* getSensorEventCode(SensorTriggerEvent event)
{
  switch (event) {
    case SENSOR1_FALLING: return "F1";
    case SENSOR2_FALLING: return "F2";
    case SENSOR1_RISING: return "R1";
    case SENSOR2_RISING: return "R2";
    default: return "";
  }
}

static void cacheSensorEvent(SensorTriggerEvent event, uint8_t triggerMask)
{
  if (sensorEventLogCount >= SENSOR_EVENT_LOG_CAPACITY) {
    for (int i = 1; i < SENSOR_EVENT_LOG_CAPACITY; i++) {
      sensorEventLog[i - 1] = sensorEventLog[i];
    }
    sensorEventLogCount = SENSOR_EVENT_LOG_CAPACITY - 1;
  }

  SensorEventLogEntry& entry = sensorEventLog[sensorEventLogCount++];
  entry.timestampMs = millis();
  strncpy(entry.eventType, getSensorEventCode(event), sizeof(entry.eventType));
  entry.eventType[sizeof(entry.eventType) - 1] = '\0';
  entry.triggerMask = triggerMask;
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
  pulseOutputEnabled = anySensorStimActive();
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

static void handleOneSensorTriggerEvent(SensorTriggerEvent event)
{
  uint8_t triggerMask = 0;

  for (int i = 0; i < NUM_HBRIDGES; i++) {
    if (triggerSelectedForEvent(i, event)) {
      triggerMask |= (1U << i);
    }
  }

  cacheSensorEvent(event, triggerMask);

  if (triggerMask & 0x01) startSensorStim(0);
  if (triggerMask & 0x02) startSensorStim(1);
}

static void handleSensorTriggerEvents(bool sensor1HighNow, bool sensor2HighNow)
{
  bool events[SENSOR_TRIGGER_EVENT_COUNT] = {
    prevSensor1High && !sensor1HighNow,
    prevSensor2High && !sensor2HighNow,
    !prevSensor1High && sensor1HighNow,
    !prevSensor2High && sensor2HighNow,
  };

  for (int eventIndex = 0; eventIndex < SENSOR_TRIGGER_EVENT_COUNT; eventIndex++) {
    if (events[eventIndex]) {
      handleOneSensorTriggerEvent((SensorTriggerEvent)eventIndex);
    }
  }
}

void updateTriggerModePingWatchdog()
{
  if (activeTriggerMode == TRIGGER_MODE_NONE) return;
  if (lastTriggerModePingTime == 0) return;

  if (millis() - lastTriggerModePingTime > TRIGGER_MODE_PING_TIMEOUT_MS) {
    stopAllTriggering();
    Serial.println("Safety: trigger mode disabled because Web UI ping timed out");
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

  if (activeTriggerMode == TRIGGER_MODE_SENSOR_TRIGGERED && sensorTriggerEnabled) {
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

  prepareTriggerMode(TRIGGER_MODE_SENSOR_TRIGGERED);

  portENTER_CRITICAL(&timerMux);
  electrodeSensorTriggerEnabled[bridgeIndex] = true;
  electrodeStimActive[bridgeIndex] = false;
  electrodeStimStartTime[bridgeIndex] = 0;
  electrodeSilentUntil[bridgeIndex] = 0;
  hBridges[bridgeIndex].enabled = false;
  hBridges[bridgeIndex].lastPulseState = -1;
  digitalWrite(hBridges[bridgeIndex].posPin, LOW);
  digitalWrite(hBridges[bridgeIndex].negPin, LOW);
  sensorTriggerEnabled = true;
  lastTriggerModePingTime = millis();
  pulseOutputEnabled = anySensorStimActive();
  portEXIT_CRITICAL(&timerMux);

  Serial.print("Web: Electrode ");
  Serial.print(bridgeIndex + 1);
  Serial.println(" sensor trigger STARTED");
}

void stopAllSensorTriggers()
{
  stopAllTriggering();
  Serial.println("Web: Sensor trigger mode DISABLED");
}

void noteTriggerModePing()
{
  lastTriggerModePingTime = millis();
}

String consumeSensorEventLogJson()
{
  String json = "{\"entries\":[";

  for (int i = 0; i < sensorEventLogCount; i++) {
    if (i > 0) json += ",";
    json += "{\"timestamp\":";
    json += String(sensorEventLog[i].timestampMs);
    json += ",\"event\":\"";
    json += sensorEventLog[i].eventType;
    json += "\",\"triggered\":";
    json += String(sensorEventLog[i].triggerMask);
    json += "}";
  }

  json += "]}";
  sensorEventLogCount = 0;
  return json;
}

void setupSensorControl()
{
  lastPressureTime = millis();
  sensorTriggerEnabled = anySensorTriggerElectrodeEnabled();
}

#include "FesController.h"

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
  pulseOutputEnabled = enabled || hBridges[0].enabled || hBridges[1].enabled;
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
    // Sensor 1 falling edge â†’ turn pulses ON
    if (prevSensor1High && !sensor1HighNow && sensorSmState == SM_WAITING) {
      sensorSmState = SM_ACTIVE;
      sensorTriggerStartTime = millis();
      setSensorTriggeredElectrodes(true);
      Serial.println("SM: Sensor1 falling edge â†’ selected electrodes ON");
    }
    // Sensor 2 rising edge â†’ turn pulses OFF
    if (!prevSensor2High && sensor2HighNow && sensorSmState == SM_ACTIVE) {
      sensorSmState = SM_WAITING;
      sensorTriggerStartTime = 0;
      setSensorTriggeredElectrodes(false);
      Serial.println("SM: Sensor2 rising edge â†’ selected electrodes OFF");
    }
    if (sensorSmState == SM_ACTIVE && sensorTriggerStartTime > 0) {
      unsigned long maxStimMs = (unsigned long)(SENSOR_TRIGGER_MAX_STIM_SECONDS * 1000.0f + 0.5f);
      if (millis() - sensorTriggerStartTime >= maxStimMs) {
        sensorSmState = SM_WAITING;
        sensorTriggerStartTime = 0;
        setSensorTriggeredElectrodes(false);
        Serial.println("SM: Maximum sensor trigger stimulation time reached â†’ selected electrodes OFF");
      }
    }
  }

  // Update states
  pressureHigh  = sensor1HighNow;
  sensor2High   = sensor2HighNow;
  prevSensor1High = sensor1HighNow;
  prevSensor2High = sensor2HighNow;
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
  pulseOutputEnabled = hBridges[0].enabled || hBridges[1].enabled;
  if (!anyElectrodeCycleEnabled()) {
    pulseCycleEnabled = false;
    cyclePhaseOn = false;
  }
  portEXIT_CRITICAL(&timerMux);

  Serial.print("Web: Electrode ");
  Serial.print(bridgeIndex + 1);
  Serial.println(" sensor trigger ENABLED");
}

void setupSensorControl()
{
  lastPressureTime = millis();
}

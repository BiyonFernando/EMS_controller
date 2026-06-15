#include "FesController.h"

float getMaxAllowedCurrent()
{
  return (0.95f * ESP32_PWM_VOLTAGE) / CURRENT_SENSE_RESISTOR;
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

void updateOvercurrentRecovery()
{
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
}

void setupSafety()
{
  attachInterrupt(digitalPinToInterrupt(hBridges[0].comparatorPin), onComparator0Trigger, FALLING);
  attachInterrupt(digitalPinToInterrupt(hBridges[1].comparatorPin), onComparator1Trigger, FALLING);
  Serial.println("Overcurrent protection enabled on GPIO6 (H-Bridge 1) and GPIO7 (H-Bridge 2)");

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
}

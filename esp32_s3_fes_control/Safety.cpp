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

static void IRAM_ATTR enableComparatorInterrupt(int bridgeIndex)
{
  if (bridgeIndex == 0) {
    attachInterrupt(digitalPinToInterrupt(hBridges[0].comparatorPin), onComparator0Trigger, FALLING);
  } else if (bridgeIndex == 1) {
    attachInterrupt(digitalPinToInterrupt(hBridges[1].comparatorPin), onComparator1Trigger, FALLING);
  }
}

static void IRAM_ATTR disableComparatorInterrupt(int bridgeIndex)
{
  detachInterrupt(digitalPinToInterrupt(hBridges[bridgeIndex].comparatorPin));
}

static void IRAM_ATTR armOvercurrentTimer(int bridgeIndex)
{
  timerWrite(overcurrentTimers[bridgeIndex], 0);
  timerAlarm(overcurrentTimers[bridgeIndex], OVERCURRENT_VERIFY_DELAY_US, false, 0);
}

static void IRAM_ATTR startOvercurrentPending(int bridgeIndex, unsigned long nowUs)
{
  HBridgeChannel& hb = hBridges[bridgeIndex];
  hb.overcurrentPending = true;
  hb.overcurrentVerifyWindow = false;
  hb.overcurrentPendingStartUs = nowUs;
  disableComparatorInterrupt(bridgeIndex);
  armOvercurrentTimer(bridgeIndex);
}

static void IRAM_ATTR confirmOvercurrent(int bridgeIndex)
{
  HBridgeChannel& hb = hBridges[bridgeIndex];
  digitalWrite(hb.posPin, LOW);
  digitalWrite(hb.negPin, LOW);
  hb.lastPulseState = -1;
  hb.overcurrentProtection = true;
  hb.overcurrentStartTime = millis();
  hb.overcurrentPending = false;
  hb.overcurrentVerifyWindow = false;
  disableComparatorInterrupt(bridgeIndex);
}

void IRAM_ATTR onComparatorTriggerBridge(int bridgeIndex) {
  HBridgeChannel& hb = hBridges[bridgeIndex];
  unsigned long nowUs = micros();

  if (hb.overcurrentProtection) {
    return;
  }

  if (!hb.overcurrentPending) {
    startOvercurrentPending(bridgeIndex, nowUs);
    return;
  }

  if (hb.overcurrentVerifyWindow) {
    if (nowUs - hb.overcurrentPendingStartUs <= OVERCURRENT_VERIFY_DELAY_US) {
      confirmOvercurrent(bridgeIndex);
    } else {
      startOvercurrentPending(bridgeIndex, nowUs);
    }
  }
}

static void IRAM_ATTR onOvercurrentTimerBridge(int bridgeIndex)
{
  HBridgeChannel& hb = hBridges[bridgeIndex];

  if (hb.overcurrentProtection) {
    return;
  }

  if (!hb.overcurrentPending) {
    return;
  }

  if (!hb.overcurrentVerifyWindow) {
    hb.overcurrentVerifyWindow = true;
    hb.overcurrentPendingStartUs = micros();
    enableComparatorInterrupt(bridgeIndex);
    armOvercurrentTimer(bridgeIndex);
    return;
  }

  hb.overcurrentPending = false;
  hb.overcurrentVerifyWindow = false;
}

void IRAM_ATTR onComparator0Trigger() {
  onComparatorTriggerBridge(0);
}

void IRAM_ATTR onComparator1Trigger() {
  onComparatorTriggerBridge(1);
}

void IRAM_ATTR onOvercurrentTimer0() {
  onOvercurrentTimerBridge(0);
}

void IRAM_ATTR onOvercurrentTimer1() {
  onOvercurrentTimerBridge(1);
}

void updateOvercurrentRecovery()
{
  for (int i = 0; i < NUM_HBRIDGES; i++) {
    if (!hBridges[i].overcurrentProtection) continue;

    unsigned long timeSinceOvercurrent = millis() - hBridges[i].overcurrentStartTime;
    if (timeSinceOvercurrent < OVERCURRENT_RECOVERY_MS) continue;

    hBridges[i].overcurrentProtection = false;
    hBridges[i].overcurrentPending = false;
    hBridges[i].overcurrentVerifyWindow = false;
    hBridges[i].lastPulseState = -1;
    enableComparatorInterrupt(i);

    Serial.print("H-Bridge ");
    Serial.print(i + 1);
    Serial.print(" overcurrent recovery after ");
    Serial.print(timeSinceOvercurrent);
    Serial.println(" ms");
  }
}

void setupSafety()
{
  overcurrentTimers[0] = timerBegin(1000000);
  overcurrentTimers[1] = timerBegin(1000000);

  if (overcurrentTimers[0] == NULL || overcurrentTimers[1] == NULL) {
    Serial.println("ERROR: Overcurrent debounce timer initialization failed!");
    while (true) {
      delay(1000);
    }
  }

  timerAttachInterrupt(overcurrentTimers[0], &onOvercurrentTimer0);
  timerAttachInterrupt(overcurrentTimers[1], &onOvercurrentTimer1);

  enableComparatorInterrupt(0);
  enableComparatorInterrupt(1);
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

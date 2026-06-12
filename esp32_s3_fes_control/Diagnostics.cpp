#include "FesController.h"

void setupDiagnostics()
{
  avgStartTime = millis();
}

void printDiagnostics()
{
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

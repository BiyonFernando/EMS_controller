#include "FesController.h"

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-S3 Boost Driver + Dual H-bridge Pulse");

  setupWebServer();
  setupPulseOutputs();
  setupSafety();
  setAllHBridgesOff();
  setupVoltageControl();
  setupPulseTimer();
  setupSensorControl();

  Serial.println("System ready. Dual H-bridge pulse running on hardware timer.");
  Serial.println("Use web interface to control cyclic stimulation and sensor-triggered stimulation.");
}

void loop()
{
  updateWebServer();
  updateFeedbackControl();
  updateTriggerModePingWatchdog();
  updatePulseCycle();
  updatePressureSensor();
  updateOvercurrentRecovery();
}

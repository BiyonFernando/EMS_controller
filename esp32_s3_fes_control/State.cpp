#include "FesController.h"

const char* ssid = "ESP32-AP";
const char* password = "123456789";

WebServer server(80);

hw_timer_t *pulseTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

HBridgeChannel hBridges[NUM_HBRIDGES] = {
  {13, 14, 6, 16, -1, false, 0, 400, false, 0.5f},
  {15, 17, 7, 18, -1, false, 0, 400, false, 0.5f},
};

float TARGET_VOLTAGE = 0.0f;
float currentDuty = 0.0f;
unsigned long lastFeedbackTime = 0;
float previousError = 0.0f;

volatile bool pulseOutputEnabled = false;

bool pulseCycleEnabled = false;
bool electrodeCycleEnabled[NUM_HBRIDGES] = {false, false};
unsigned long cycleLastToggle = 0;
bool cyclePhaseOn = false;

float SENSOR1_THRESHOLD_PERCENT = 50.0f;
float sensor1Percent = 0.0f;
bool sensor1High = false;
bool prevSensor1High = false;
unsigned long lastPressureTime = 0;

float SENSOR2_THRESHOLD_PERCENT = 50.0f;
float sensor2Percent = 0.0f;
bool sensor2High = false;
bool prevSensor2High = false;

bool sensorTriggerEnabled = false;
bool electrodeSensorTriggerEnabled[NUM_HBRIDGES] = {false, false};
bool electrodeTriggerEvents[NUM_HBRIDGES][SENSOR_TRIGGER_EVENT_COUNT] = {
  {false, false, false, false},
  {false, false, false, false},
};
bool electrodeStimActive[NUM_HBRIDGES] = {false, false};
unsigned long electrodeStimStartTime[NUM_HBRIDGES] = {0, 0};
unsigned long electrodeSilentUntil[NUM_HBRIDGES] = {0, 0};
float electrodeStimDurationSeconds[NUM_HBRIDGES] = {0.55f, 0.55f};
float electrodeSilentSeconds[NUM_HBRIDGES] = {1.05f, 1.05f};

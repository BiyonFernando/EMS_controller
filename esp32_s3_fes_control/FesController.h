#pragma once

#include <Arduino.h>
#include <WebServer.h>

extern const char* ssid;
extern const char* password;

extern WebServer server;

extern hw_timer_t *pulseTimer;
extern portMUX_TYPE timerMux;

constexpr int PIN_IN_PLUS  = 11;
constexpr int PIN_IN_MINUS = 12;
constexpr int PIN_VSENSE   = 1;
constexpr int PIN_SENSOR_1 = 2;
constexpr int PIN_SENSOR_2 = 3;

constexpr int NUM_HBRIDGES = 2;

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

extern HBridgeChannel hBridges[NUM_HBRIDGES];

constexpr uint32_t PWM_FREQ_HZ = 30000;
constexpr uint8_t PWM_RES_BITS = 10;

constexpr float R_TOP = 1000000.0f;
constexpr float R_BOTTOM = 10000.0f;

extern float TARGET_VOLTAGE;
constexpr unsigned long FEEDBACK_INTERVAL_MS = 100;
constexpr float Kp = 0.001f;
constexpr float Kd = 0.000f;

extern float currentDuty;
extern unsigned long lastFeedbackTime;
extern float previousError;

constexpr unsigned long PULSE_PERIOD_US = 25000;
constexpr unsigned long PULSE_GAP_US = 100;
constexpr unsigned long PULSE_WIDTH_MIN_US = 10;
constexpr unsigned long PULSE_WIDTH_MAX_US = (PULSE_PERIOD_US - PULSE_GAP_US) / 2;

extern volatile bool pulseOutputEnabled;

extern bool pulseCycleEnabled;
extern bool electrodeCycleEnabled[NUM_HBRIDGES];
extern unsigned long cycleLastToggle;
extern bool cyclePhaseOn;
constexpr unsigned long CYCLE_ON_MS = 550;
constexpr unsigned long CYCLE_OFF_MS = 1050;

extern float SENSOR1_THRESHOLD_PERCENT;
constexpr unsigned long PRESSURE_SAMPLE_MS = 10;
extern float sensor1Percent;
extern bool sensor1High;
extern bool prevSensor1High;
extern unsigned long lastPressureTime;

extern float SENSOR2_THRESHOLD_PERCENT;
extern float sensor2Percent;
extern bool sensor2High;
extern bool prevSensor2High;

enum SensorTriggerEvent {
  SENSOR1_FALLING = 0,
  SENSOR2_FALLING = 1,
  SENSOR1_RISING = 2,
  SENSOR2_RISING = 3,
  SENSOR_TRIGGER_EVENT_COUNT = 4
};

extern bool sensorTriggerEnabled;
extern bool electrodeSensorTriggerEnabled[NUM_HBRIDGES];
extern bool electrodeTriggerEvents[NUM_HBRIDGES][SENSOR_TRIGGER_EVENT_COUNT];
extern bool electrodeStimActive[NUM_HBRIDGES];
extern unsigned long electrodeStimStartTime[NUM_HBRIDGES];
extern unsigned long electrodeSilentUntil[NUM_HBRIDGES];
extern float electrodeStimDurationSeconds[NUM_HBRIDGES];
extern float electrodeSilentSeconds[NUM_HBRIDGES];

constexpr unsigned long OVERCURRENT_RECOVERY_MS = 5000;
constexpr int COMPARATOR_DEBOUNCE_CHECKS = 2;
constexpr int COMPARATOR_DEBOUNCE_DELAY_US = 2;

constexpr float CURRENT_SENSE_RESISTOR = 16.5f;
constexpr float ESP32_PWM_VOLTAGE = 3.3f;
constexpr uint32_t IMAX_PWM_FREQ = 25000;
constexpr uint8_t IMAX_PWM_RES = 10;

float getMaxAllowedCurrent();
void setDuty(float duty);
void setMaxCurrent(int bridgeIndex, float current);
float readOutputVoltage();
void updateFeedbackControl();
void setupVoltageControl();

void updatePressureSensor();
void startElectrodeSensorTrigger(int bridgeIndex);
void stopAllSensorTriggers();
void setupSensorControl();

void setHBridgeOff(HBridgeChannel& hb);
void setAllHBridgesOff();
void setPulseWidth(int bridgeIndex, unsigned long widthUs);
bool anyElectrodeCycleEnabled();
void startElectrodeCycle(int bridgeIndex);
void updatePulseCycle();
void setupPulseOutputs();
void setupPulseTimer();
void IRAM_ATTR onPulseTimer();

void IRAM_ATTR onComparator0Trigger();
void IRAM_ATTR onComparator1Trigger();
void updateOvercurrentRecovery();
void setupSafety();

void handleRoot();
void handleCycleElectrode1On();
void handleCycleElectrode2On();
void handleCycleOff();
void handleSensorTriggerElectrode1Start();
void handleSensorTriggerElectrode2Start();
void handleSensorTriggerOff();
void handleSet();
void handleStatus();
void setupWebServer();
void updateWebServer();

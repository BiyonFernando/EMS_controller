#include "FesController.h"

void setDuty(float duty)
{
  if (duty < 0.0f) duty = 0.0f;
  if (duty > 0.95f) duty = 0.95f;

  currentDuty = duty;

  uint32_t maxDuty = (1U << PWM_RES_BITS) - 1U;
  uint32_t dutyCount = (uint32_t)(duty * maxDuty + 0.5f);
  ledcWrite(PIN_IN_PLUS, dutyCount);
}

float readOutputVoltage()
{
  uint32_t adc_mV = analogReadMilliVolts(PIN_VSENSE);
  float v_adc = adc_mV / 1000.0f;
  float v_out = v_adc * (R_TOP + R_BOTTOM) / R_BOTTOM;
  return v_out;
}

void updateFeedbackControl()
{
  if (millis() - lastFeedbackTime < FEEDBACK_INTERVAL_MS) {
    return;
  }
  
  unsigned long currentTime = millis();
  float dt = (currentTime - lastFeedbackTime) / 1000.0f; // Convert to seconds
  lastFeedbackTime = currentTime;

  float vout = readOutputVoltage();
  float error = TARGET_VOLTAGE - vout;
  
  // Calculate derivative term
  float derivative = (error - previousError) / dt;
  
  // PD control output
  float controlOutput = Kp * error + Kd * derivative;
  
  // Update duty cycle
  setDuty(currentDuty + controlOutput);
  
  // Store error for next iteration
  previousError = error;
  
  // Optional: print for debugging
  // Serial.print("Vout = ");
  // Serial.print(vout, 2);
  // Serial.print(" V, Error = ");
  // Serial.print(error, 2);
  // Serial.print(", Control = ");
  // Serial.println(controlOutput, 4);
}

void setupVoltageControl()
{
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VSENSE, ADC_11db);
  analogSetPinAttenuation(PIN_SENSOR_1, ADC_11db);
  analogSetPinAttenuation(PIN_SENSOR_2, ADC_11db);

  if (!ledcAttach(PIN_IN_PLUS, PWM_FREQ_HZ, PWM_RES_BITS)) {
    Serial.println("LEDC attach failed");
    while (true) {
      delay(1000);
    }
  }

  setDuty(0.0f);
  delay(1000);

  lastFeedbackTime = millis();
}

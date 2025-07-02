#include "led_task.h"

#include <Arduino.h>

LedTask::LedTask(bool reversed) : m_reversed(reversed), m_isOn(false) {}

LedTask::~LedTask() {}

void LedTask::setup() {

  Serial.println("setting up led task...");

  pinMode(LED_PIN_PIN, OUTPUT);
  ledOff();

  Serial.println("led task started.");
}

unsigned long LedTask::loop(MicroTasks::WakeReason reason) {

  // blinking in 1Hz
  // Serial.println("led loop begin.");
  if (m_isOn)
    ledOff();
  else
    ledOn();
  // Serial.println("led loop end.");
  return 1000;
}
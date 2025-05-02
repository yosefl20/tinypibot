#include "led_task.h"
#include "board_def.h"

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
  if (m_isOn)
    ledOff();
  else
    ledOn();

  return 1000;
}
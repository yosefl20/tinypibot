#include "battery_task.h"
#include "board_def.h"

#include <Arduino.h>

BatteryTask::BatteryTask() {}

BatteryTask::~BatteryTask() {}

void BatteryTask::setup() {

  Serial.println("setting up battery task...");

  pinMode(BATT_PIN_PIN, INPUT);
  m_batteryVoltage = analogRead(BATT_PIN_PIN);

  Serial.println("battery task started.");
}

unsigned long BatteryTask::loop(MicroTasks::WakeReason reason) {
  m_batteryVoltage = analogRead(BATT_PIN_PIN);
  return 1000;
}
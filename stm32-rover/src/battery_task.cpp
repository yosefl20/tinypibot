#include "battery_task.h"
#include "board_def.h"

#include "msgs.h"
#include "uart_task.h"

extern UartTask uartTask;

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
  Serial.print("battery: ");
  Serial.println(m_batteryVoltage * 12.6f);
  uartTask.send(new BatteryMessage(m_batteryVoltage));
  return 1000;
}
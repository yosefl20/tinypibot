#include "ultrasonic_task.h"
#include "board_def.h"

UltrasonicTask::UltrasonicTask()
    : m_sensor1(ULTRA_SONIC_TRIG_PIN, ULTRA_SONIC_ECHO_PIN) {}

UltrasonicTask::~UltrasonicTask() {}

void UltrasonicTask::setup() {

  Serial.println("setting up ultrasonic task...");

  Serial.println("ultrasonic started.");
}

unsigned long UltrasonicTask::loop(MicroTasks::WakeReason reason) {
  return INFINITY;
}
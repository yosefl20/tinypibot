#include "ultrasonic_task.h"
#include "board_def.h"
#include "msgs.h"
#include "uart_task.h"

extern UartTask uartTask;

UltrasonicTask::UltrasonicTask()
    : m_sensor1(ULTRA_SONIC_TRIG_PIN, ULTRA_SONIC_ECHO_PIN), m_distance(1000) {}

UltrasonicTask::~UltrasonicTask() {}

void UltrasonicTask::setup() {

  Serial.println("setting up ultrasonic task...");

  Serial.println("ultrasonic started.");
}

unsigned long UltrasonicTask::loop(MicroTasks::WakeReason reason) {
  // Serial.println("ultrasonic loop begin.");
  m_distance = m_sensor1.read();
  uartTask.send(new DistanceMessage(m_distance, 0, 0, 0));
  // Serial.println("ultrasonic loop end.");
  return 200; // 5hz
}
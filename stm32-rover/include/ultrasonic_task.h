#pragma once
#ifndef _ULTRASONIC_TASK_H_
#define _ULTRASONIC_TASK_H_

#include <Arduino.h>

#include "MicroTasks.h"
#include <Ultrasonic.h>

/**
 * 超声波管理任务
 */
class UltrasonicTask : public MicroTasks::Task {
public:
  UltrasonicTask();
  ~UltrasonicTask();

  void setup();
  unsigned long loop(MicroTasks::WakeReason reason);

  inline unsigned int distance() const { return m_distance; }

private:
  Ultrasonic m_sensor1;
  unsigned int m_distance;
};

#endif //_ULTRASONIC_TASK_H_
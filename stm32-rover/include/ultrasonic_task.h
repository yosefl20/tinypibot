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

private:
  Ultrasonic m_sensor1;
};

#endif //_ULTRASONIC_TASK_H_
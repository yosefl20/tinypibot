#pragma once
#ifndef _BATTERY_TASK_H_
#define _BATTERY_TASK_H_

#include "MicroTasks.h"

/**
 * 电池管理任务
 */
class BatteryTask : public MicroTasks::Task {
public:
  BatteryTask();
  ~BatteryTask();

  void setup();
  unsigned long loop(MicroTasks::WakeReason reason);

  inline int batteryVoltage() const { return m_batteryVoltage; }

private:
  int m_batteryVoltage;
};

#endif //_BATTERY_TASK_H_
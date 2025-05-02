#pragma once
#ifndef _I2C_TASK_H_
#define _I2C_TASK_H_

#include "MicroTasks.h"

/**
 * I2C通讯任务
 */
class I2cTask : public MicroTasks::Task {
public:
  I2cTask();
  ~I2cTask();

  void setup();
  unsigned long loop(MicroTasks::WakeReason reason);

private:
  static void onI2CRequestEvent();
  static void onI2CData(int bytesCount);
};

#endif //_I2C_TASK_H_
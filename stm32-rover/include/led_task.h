#pragma once
#ifndef _LED_TASK_H_
#define _LED_TASK_H_

#include "board_def.h"

#include "MicroTasks.h"

/**
 * 內建LED任务
 */
class LedTask : public MicroTasks::Task {
public:
  LedTask(bool reversed);
  ~LedTask();

  void setup();
  unsigned long loop(MicroTasks::WakeReason reason);

  inline void ledOn() {
    m_isOn = true;
    digitalWrite(LED_PIN_PIN, m_reversed ? LOW : HIGH);
  }
  inline void ledOff() {
    m_isOn = false;
    digitalWrite(LED_PIN_PIN, m_reversed ? HIGH : LOW);
  }

private:
  bool m_reversed;
  bool m_isOn;
};

#endif //_LED_TASK_H_
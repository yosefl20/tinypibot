#pragma once
#ifndef _UART_TASK_H_
#define _UART_TASK_H_

#include "MicroTasks.h"

#define UART_PACKET_LEN 12

/**
 * UART通讯任务
 */
class UartTask : public MicroTasks::Task {
public:
  UartTask();
  ~UartTask();

  void setup();
  unsigned long loop(MicroTasks::WakeReason reason);

protected:
  void sendPacket(uint8_t cmd, int32_t d0, int32_t d1);
  void sendPacket(uint8_t cmd, float f0, float f1);
  void sendPacket(uint8_t cmd, int16_t d0, int16_t d1, int16_t d2, int16_t d3);

private:
  uint8_t m_recvBuf[UART_PACKET_LEN];
  uint8_t m_recvLen;
  HardwareSerial SerialPi;
};

#endif //_UART_TASK_H_
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

  inline HardwareSerial &getSerialPi() { return SerialPi; }

private:
  HardwareSerial SerialPi;
};

class UartRecvTask : public MicroTasks::Task {
public:
  UartRecvTask(HardwareSerial &serial);
  ~UartRecvTask();

  void setup();
  unsigned long loop(MicroTasks::WakeReason reason);

protected:
  void recvPackets();

private:
  uint8_t m_recvBuf[UART_PACKET_LEN];
  uint8_t m_recvLen;
  HardwareSerial &SerialPi;
};

#endif //_UART_TASK_H_
#pragma once
#ifndef _ROVER_TASK_H_
#define _ROVER_TASK_H_

#include <Arduino.h>
#include <PID_v1.h>
#include <PWMDcMotor.h>
#include <STM32encoder.h>

#include "MicroTasks.h"

/**
 * 底盘管理任务
 */
class RoverTask : public MicroTasks::Task {
public:
  RoverTask();
  ~RoverTask();

  void setup();
  unsigned long loop(MicroTasks::WakeReason reason);

  inline STM32encoder &leftEncoder() { return m_encoderA; }
  inline STM32encoder &rightEncoder() { return m_encoderB; }

  inline int16_t leftSpeed() const { return m_leftSpeed; }
  inline int16_t rightSpeed() const { return m_rightSpeed; }

  inline void enablePID(bool b) { m_activePID = b; }
  inline bool isPidEnabled() const { return m_activePID; }
  inline void setPIDParams(double p, double i, double d) {
    m_Kp = p;
    m_Kd = d;
    m_Ki = i;
  }

  inline void setPIDkP(double v) { m_Kp = v; }
  inline void setPIDkI(double v) { m_Ki = v; }
  inline void setPIDkD(double v) { m_Kd = v; }

protected:
  void resetEncoderPos();

  void initPID();
  void updatePID();
  void applyPID();

private:
  PWMDcMotor m_motorA;
  PWMDcMotor m_motorB;
  STM32encoder m_encoderA;
  STM32encoder m_encoderB;
  int8_t m_leftThrottle;
  int8_t m_rightThrottle;
  float m_leftSpeed;
  float m_rightSpeed;
  unsigned long m_tmUpdateSpeed;

  // PID状态
  bool m_activePID;
  // Define Variables we'll be connecting to
  double m_SetpointA, m_InputA, m_OutputA, m_PowA;
  double m_SetpointB, m_InputB, m_OutputB, m_PowB;
  const long m_cycleTM;
  // Specify the links and initial tuning parameters
  double m_Kp, m_Ki, m_Kd;
  PID m_pidA;
  PID m_pidB;
};

#endif //_ROVER_TASK_H_
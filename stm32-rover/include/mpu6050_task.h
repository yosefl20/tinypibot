#pragma once
#ifndef _MPU_TASK_H_
#define _MPU_TASK_H_

#include "board_def.h"

#include "MicroTasks.h"

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
// #include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

class Mpu6050Task : public MicroTasks::Task {
public:
  Mpu6050Task();
  ~Mpu6050Task();

  void setup();
  unsigned long loop(MicroTasks::WakeReason reason);

  inline bool isReady() const { return m_dmpReady; }
  inline uint8_t devStatus() const { return m_devStatus; }

  inline const Quaternion &getQuaternion() const { return m_q; }
  inline const VectorFloat &getAccelReading() const { return m_accel; }
  inline const VectorFloat &getGyroReading() const { return m_gyro; }
  inline const float *getEulerAngle() const { return m_euler; }
  inline const float *getYawPitchRoll() const { return m_ypr; }

private:
  MPU6050 m_mpu;

  float m_accelScalar;
  float m_gyroScalar;
  VectorFloat m_accel;
  VectorFloat m_gyro;

  // MPU control/status vars
  bool m_dmpReady; // set true if DMP init was successful
  // uint8_t m_mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t m_devStatus;      // return status after each device operation (0 =
                            // success, !0 = error)
  uint16_t m_packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t m_fifoCount;     // count of all bytes currently in FIFO
  uint8_t m_fifoBuffer[64]; // FIFO storage buffer

  // orientation/motion vars
  Quaternion m_q;   // [w, x, y, z]         quaternion container
  VectorInt16 m_aa; // [x, y, z]            accel sensor measurements, -2g ~ +2g
  VectorInt16 m_ag; // [x, y, z]            gyro sensor measurements, -250 ~
                    // +250 degs/sec
  VectorInt16
      m_aaReal; // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16
      m_aaWorld; // [x, y, z]            world-frame accel sensor measurements
  VectorFloat m_gravity; // [x, y, z]            gravity vector
  float m_euler[3];      // [psi, theta, phi]    Euler angle container
  float m_ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity
                  // vector
};

#endif // _MPU_TASK_H_
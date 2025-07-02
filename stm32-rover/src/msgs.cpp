#include "msgs.h"

LinearSpeedMessage::LinearSpeedMessage(int motorId, float linearSpeed)
    : m_motorId(motorId), m_linearSpeed(linearSpeed) {}

ThrottleMessage::ThrottleMessage(ThrottleMessage::Type type, int8_t throttle)
    : m_type(type), m_throttle(throttle) {}

EncoderPositionMessage::EncoderPositionMessage(int32_t left, int32_t right)
    : m_left(left), m_right(right) {}

DistanceMessage::DistanceMessage(int16_t distance0, int16_t distance1,
                                 int16_t distance2, int16_t distance3) {
  m_distances[0] = distance0;
  m_distances[1] = distance1;
  m_distances[2] = distance2;
  m_distances[3] = distance3;
}

BatteryMessage::BatteryMessage(int32_t mV) {
  m_mV = mV * 12.6f;
  m_percent = mV / 10.0f; // 12.6V MAX
}

ImuMessage::ImuMessage(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                       int16_t accel_range, int16_t gyro_x, int16_t gyro_y,
                       int16_t gyro_z, int16_t gyro_range, int16_t q_w,
                       int16_t q_x, int16_t q_y, int16_t q_z) {
  m_accel[0] = accel_x;
  m_accel[1] = accel_y;
  m_accel[2] = accel_z;
  m_accel[3] = accel_range;
  m_gyro[0] = gyro_x;
  m_gyro[1] = gyro_y;
  m_gyro[2] = gyro_z;
  m_gyro[3] = gyro_range;
  m_q[0] = q_w;
  m_q[1] = q_x;
  m_q[2] = q_y;
  m_q[3] = q_z;
}

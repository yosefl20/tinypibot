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
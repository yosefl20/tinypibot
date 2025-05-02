#include "msgs.h"

LinearSpeedMessage::LinearSpeedMessage(int motorId, float linearSpeed)
    : m_motorId(motorId), m_linearSpeed(linearSpeed) {}

ThrottleMessage::ThrottleMessage(ThrottleMessage::Type type, int8_t throttle)
    : m_type(type), m_throttle(throttle) {}
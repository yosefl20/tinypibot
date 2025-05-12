#pragma once
#ifndef _MSGS_H_
#define _MSGS_H_

#include "MicroTasks.h"
#include "MicroTasksMessage.h"

template <uint32_t t_message_id>
class SimpleMessage : public MicroTasks::Message {

protected:
  virtual void receive();

public:
  SimpleMessage();

  static const uint32_t ID = t_message_id;
};

template <uint32_t t_message_id>
SimpleMessage<t_message_id>::SimpleMessage() : Message(ID) {}

template <uint32_t t_message_id> void SimpleMessage<t_message_id>::receive() {}

// ----------------------------------------------------------------
typedef SimpleMessage<1001> ResetEncoderPositionMessage;

// ----------------------------------------------------------------
class LinearSpeedMessage : public SimpleMessage<1002> {
public:
  LinearSpeedMessage(int motorId, float linearSpeed);

  int m_motorId;
  float m_linearSpeed;
};

// ----------------------------------------------------------------
class ThrottleMessage : public SimpleMessage<1004> {
public:
  enum Type {
    TMT_NONE = 0,
    TMT_LEFT = 1,
    TMT_RIGHT = 2,
    TMT_BOTH = 3,
  };

  ThrottleMessage(Type type, int8_t throttle);

  Type m_type;
  int8_t m_throttle;
};

// ----------------------------------------------------------------
class EncoderPositionMessage : public SimpleMessage<1005> {
public:
  EncoderPositionMessage(int32_t left, int32_t right);

  int32_t m_left;
  int32_t m_right;
};

// ----------------------------------------------------------------
class DistanceMessage : public SimpleMessage<1006> {
public:
  DistanceMessage(int16_t distance0, int16_t distance1, int16_t distance2,
                  int16_t distance3);

  int16_t m_distances[4];
};

// ----------------------------------------------------------------
class BatteryMessage : public SimpleMessage<1007> {
public:
  BatteryMessage(int32_t mV);

  float m_mV;
  float m_percent;
};

#endif //_MSGS_H_
#include "rover_task.h"
#include "board_def.h"
#include <PWMDcMotor.hpp>

#include "msgs.h"
#include "uart_task.h"

#define WHEEL_SIZE 68.0  // mm
#define BODY_WIDTH 150.0 // mm

// #define ENABLE_DEBUG_MESSAGE

extern UartTask uartTask;

RoverTask::RoverTask()
    : m_motorA(TB6612_AIN1_PIN, TB6612_AIN2_PIN, TB6612_ENA_PIN),
      m_motorB(TB6612_BIN2_PIN, TB6612_BIN1_PIN, TB6612_ENB_PIN),
      m_encoderA(MA_TIMER_ID), m_encoderB(MB_TIMER_ID), m_leftThrottle(0),
      m_rightThrottle(0), m_leftSpeed(0), m_rightSpeed(0), m_cycleTM(100),
      // Specify the links and initial tuning parameters
      // m_Kp(0.1), m_Ki(0.03), m_Kd(0.002),
      m_Kp(0.05), m_Ki(0.2), m_Kd(0.001),
      m_pidA(&m_InputA, &m_OutputA, &m_SetpointA, m_Kp, m_Ki, m_Kd, DIRECT),
      m_pidB(&m_InputB, &m_OutputB, &m_SetpointB, m_Kp, m_Ki, m_Kd, DIRECT) {}

RoverTask::~RoverTask() {}

void RoverTask::setup() {

  Serial.println("setting up rover task...");

  // motors
  m_encoderA.circular(true);
  m_encoderB.circular(true);
  resetEncoderPos();

  // Speed control
  pinMode(TB6612_ENA_PIN, OUTPUT);
  pinMode(TB6612_ENB_PIN, OUTPUT);
  analogWrite(TB6612_ENA_PIN, 0);
  analogWrite(TB6612_ENB_PIN, 0);

  // Outputs
  pinMode(TB6612_AIN1_PIN, OUTPUT);
  pinMode(TB6612_AIN2_PIN, OUTPUT);
  pinMode(TB6612_BIN1_PIN, OUTPUT);
  pinMode(TB6612_BIN2_PIN, OUTPUT);

  m_motorA.stop();
  m_motorB.stop();

  // pid
  initPID();

  m_tmUpdateSpeed = millis();

  Serial.println("rover task started.");
}

unsigned long RoverTask::loop(MicroTasks::WakeReason reason) {

  // Serial.println("RoverTask loop begin.");
#ifdef ENABLE_DEBUG_MESSAGE
  if (reason != WakeReason_Scheduled) {
    Serial.print("RoverTask woke: ");
    Serial.println(WakeReason_Scheduled == reason ? "WakeReason_Scheduled"
                   : WakeReason_Event == reason   ? "WakeReason_Event"
                   : WakeReason_Message == reason ? "WakeReason_Message"
                   : WakeReason_Manual == reason  ? "WakeReason_Manual"
                                                  : "UNKNOWN");
  }
#endif

  if (WakeReason_Message == reason) {
    MicroTasks::Message *msg;
    if (this->receive(msg)) {
      Serial.print("Got message ");
      switch (msg->id()) {
      case ResetEncoderPositionMessage::ID: {
        Serial.println("ResetEncoderPositionMessage");
        resetEncoderPos();
      } break;
      case LinearSpeedMessage::ID: {
        m_pidA.SetMode(AUTOMATIC);
        m_pidB.SetMode(AUTOMATIC);

        LinearSpeedMessage *_msg = (LinearSpeedMessage *)msg;
        Serial.print("LinearSpeedMessage, motor = ");
        Serial.print(_msg->m_motorId);
        Serial.print(", speed = ");
        Serial.println(_msg->m_linearSpeed);
        if (_msg->m_motorId == 0)
          m_SetpointA = _msg->m_linearSpeed;
        else if (_msg->m_motorId == 1)
          m_SetpointB = _msg->m_linearSpeed;
      } break;
      case ThrottleMessage::ID: {
        m_pidA.SetMode(MANUAL);
        m_pidB.SetMode(MANUAL);
        // m_PowA = m_PowB = m_OutputA = m_OutputB = 0;
        ThrottleMessage *_msg = (ThrottleMessage *)msg;
        Serial.print("ThrottleMessage, type = ");
        Serial.print(_msg->m_type);
        Serial.print("throttle = ");
        Serial.println(_msg->m_throttle);
        if (_msg->m_type == ThrottleMessage::TMT_LEFT ||
            _msg->m_type == ThrottleMessage::TMT_BOTH) {
          m_leftThrottle = _msg->m_throttle;
        }
        if (_msg->m_type == ThrottleMessage::TMT_RIGHT ||
            _msg->m_type == ThrottleMessage::TMT_BOTH) {
          m_rightThrottle = _msg->m_throttle;
        }
        m_OutputA = m_leftThrottle;
        m_OutputB = m_rightThrottle;
      }
      default: {
        Serial.println("UNKNOWN");
      } break;
      }
      delete msg;
    }
  }

  // update speed
  unsigned long t = millis();
  if (t > m_tmUpdateSpeed) {
    float dt = t - m_tmUpdateSpeed;
    int32_t left_pos = m_encoderA.pos();
    int32_t right_pos = m_encoderB.pos();
    // send pos to uart
    uartTask.send(new EncoderPositionMessage(left_pos, right_pos));
    m_leftSpeed = (float)left_pos * 1000.0f / dt;
    m_rightSpeed = (float)right_pos * (-1000.0f) / dt;
    m_encoderA.pos(0);
    m_encoderB.pos(0);
    m_tmUpdateSpeed = t;

    if (fabs(m_leftSpeed) > 1e-5 || fabs(m_rightSpeed) > 1e-5) {
      Serial.print("encoder speed = ");
      Serial.print(m_leftSpeed);
      Serial.print(" / ");
      Serial.println(m_rightSpeed);
    }
  }

  updatePID();
  applyPID();

  // Serial.println("RoverTask loop end.");
  return m_cycleTM;
}

void RoverTask::resetEncoderPos() {
  m_encoderA.pos(0);
  m_encoderB.pos(0);
}

void RoverTask::initPID() {

  m_InputA = m_InputB = 0;
  m_SetpointA = m_SetpointB = 0;
  m_pidA.SetOutputLimits(-255, 255);
  m_pidA.SetMode(AUTOMATIC);
  m_pidB.SetOutputLimits(-255, 255);
  m_pidB.SetMode(AUTOMATIC);
  m_OutputA = m_OutputB = 0;
}

void RoverTask::updatePID() {

#ifdef ENABLE_DEBUG_MESSAGE
  Serial.println("RoverTask updatePID begin");
#endif

  m_InputA = m_leftSpeed;
  m_InputB = m_rightSpeed;

#ifdef ENABLE_DEBUG_MESSAGE
  Serial.print("speed = ");
  Serial.print(m_InputA);
  Serial.print(" / ");
  Serial.println(m_InputB);
#endif

  m_pidA.Compute();
  m_pidB.Compute();

#ifdef ENABLE_DEBUG_MESSAGE
  // debug output
  Serial.print("SetPoint = (");
  Serial.print(m_SetpointA);
  Serial.print(" , ");
  Serial.print(m_SetpointB);
  Serial.print(")  /  Input = (");
  Serial.print(m_InputA);
  Serial.print(" , ");
  Serial.print(m_InputB);
  Serial.print(")  /  Output = (");
  Serial.print(m_OutputA);
  Serial.print(" , ");
  Serial.print(m_OutputB);
  Serial.println(")");
  Serial.println("RoverTask updatePID done");
#endif
}

void RoverTask::applyPID() {

  if (fabs(m_SetpointA) < 1e-5 && fabs(m_SetpointB) < 1e-5) {
    m_pidA.SetMode(MANUAL);
    m_pidB.SetMode(MANUAL);
    m_OutputA = m_OutputB = 0;
#ifdef ENABLE_DEBUG_MESSAGE
    Serial.println("rover stopped");
#endif
  }

  if (fabs(m_OutputA) < 1e-5 && fabs(m_OutputB) < 1e-5) {
    m_motorA.stop();
    m_motorB.stop();
  } else {
    m_motorA.setSpeedPWMAndDirection(m_OutputA * 0.702f);
    m_motorB.setSpeedPWMAndDirection(m_OutputB);
  }
}

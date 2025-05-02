#include "rover_task.h"
#include "board_def.h"
#include <PWMDcMotor.hpp>

#include "msgs.h"

#define WHEEL_SIZE 68.0  // mm
#define BODY_WIDTH 150.0 // mm

RoverTask::RoverTask()
    : m_motorA(TB6612_AIN1_PIN, TB6612_AIN2_PIN, TB6612_ENA_PIN),
      m_motorB(TB6612_BIN2_PIN, TB6612_BIN1_PIN, TB6612_ENB_PIN),
      m_encoderA(MA_TIMER_ID), m_encoderB(MB_TIMER_ID), m_leftThrottle(0),
      m_rightThrottle(0), m_leftSpeed(0), m_rightSpeed(0), m_cycleTM(100),
      // Specify the links and initial tuning parameters
      m_Kp(0.03), m_Ki(0.02), m_Kd(0.002),
      m_pidA(&m_InputA, &m_OutputA, &m_SetpointA, m_Kp, m_Ki, m_Kd, DIRECT),
      m_pidB(&m_InputB, &m_OutputB, &m_SetpointB, m_Kp, m_Ki, m_Kd, DIRECT),
      m_activePID(false) {}

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

  if (reason != WakeReason_Scheduled) {
    Serial.print("RoverTask woke: ");
    Serial.println(WakeReason_Scheduled == reason ? "WakeReason_Scheduled"
                   : WakeReason_Event == reason   ? "WakeReason_Event"
                   : WakeReason_Message == reason ? "WakeReason_Message"
                   : WakeReason_Manual == reason  ? "WakeReason_Manual"
                                                  : "UNKNOWN");
  }

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
        m_activePID = true;
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
        m_activePID = false;
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
        if (!m_activePID) {
          m_motorA.setSpeedPWMAndDirection(m_leftThrottle);
          m_motorB.setSpeedPWMAndDirection(m_rightThrottle);
        }
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
    m_leftSpeed = (float)m_encoderA.pos() * 1000.0f / dt;
    m_rightSpeed = (float)m_encoderB.pos() * (-1000.0f) / dt;
    m_encoderA.pos(0);
    m_encoderB.pos(0);
    m_tmUpdateSpeed = t;

    if (fabs(m_leftSpeed) > 1e-5 || fabs(m_rightSpeed) > 1e-5) {
      Serial.print("encoder speed = ");
      Serial.print(m_leftSpeed);
      Serial.print(" / ");
      Serial.println(m_rightSpeed);
    }

    if (m_activePID) {
      updatePID();
      applyPID();
    }
  }

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
  m_PowA = m_PowB = m_OutputA = m_OutputB = 0;
}

void RoverTask::updatePID() {

  Serial.println("RoverTask updatePID begin");

  m_InputA = m_leftSpeed;
  m_InputB = m_rightSpeed;

  Serial.print("speed = ");
  Serial.print(m_InputA);
  Serial.print(" / ");
  Serial.println(m_InputB);

  m_pidA.Compute();
  m_pidB.Compute();
  m_PowA += m_OutputA;
  m_PowA = max(m_PowA, -255.0);
  m_PowA = min(m_PowA, 255.0);

  m_PowB += m_OutputB;
  m_PowB = max(m_PowB, -255.0);
  m_PowB = min(m_PowB, 255.0);

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
  Serial.print(")  /  Pow = (");
  Serial.print(m_PowA);
  Serial.print(" , ");
  Serial.print(m_PowB);
  Serial.println(")");
  Serial.println("RoverTask updatePID done");
}

void RoverTask::applyPID() {

  if (m_InputA == 0 && m_InputB == 0 && m_SetpointA == 0 && m_SetpointB == 0) {
    m_activePID = false;
    m_motorA.stop();
    m_motorB.stop();
    Serial.println("rover stopped");
    return;
  }

  m_motorA.setSpeedPWMAndDirection(m_PowA);
  m_motorB.setSpeedPWMAndDirection(m_PowB);

  const float tmp = M_PI * WHEEL_SIZE / 60.0;
  float linear = (m_InputA + m_InputB) * tmp;
  float angular = 0.0;

  // publish odom
  // g_bleTask.send(new OdomMessage(linear, angular));
  // g_spiSlaveTask.send(new OdomMessage(linear, angular));
}

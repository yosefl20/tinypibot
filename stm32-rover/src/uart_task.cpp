#include "uart_task.h"
#include "board_def.h"

#include <Arduino.h>
#include <Wire.h>

#include "battery_task.h"
#include "led_task.h"
#include "rover_task.h"
#include "ultrasonic_task.h"

#include "msgs.h"

#define PACKET_HEADER 0xE7

extern BatteryTask batteryTask;
extern UartTask uartTask;
extern LedTask ledTask;
extern RoverTask roverTask;
extern UltrasonicTask ultrasonicTask;

#pragma pack(1)
typedef struct t_uart_packet {
  uint8_t header;
  uint8_t cmd;
  union {
    uint8_t raw[8];
    int32_t i32Array[2];
    int16_t i16Array[4];
    float_t f32Array[2];
  } payload;
  uint8_t checkSum;
  uint8_t padding;
} UartPacket;

static uint8_t calcCheckSum(const UartPacket &packet) {
  int32_t val = packet.header + packet.cmd + packet.payload.raw[0] +
                packet.payload.raw[1] + packet.payload.raw[2] +
                packet.payload.raw[3] + packet.payload.raw[4] +
                packet.payload.raw[5] + packet.payload.raw[6] +
                packet.payload.raw[7];
  return val & 0xff;
}

static void debugPrintUartPacket(const UartPacket &packet) {
  Serial.println("---------------------");
  Serial.print(" header: 0x");
  Serial.println(packet.header, HEX);
  Serial.print("    cmd: 0x");
  Serial.println(packet.cmd, HEX);
  Serial.print("payload: ");
  Serial.print(packet.payload.raw[0], HEX);
  Serial.print(" ");
  Serial.print(packet.payload.raw[1], HEX);
  Serial.print(" ");
  Serial.print(packet.payload.raw[2], HEX);
  Serial.print(" ");
  Serial.print(packet.payload.raw[3], HEX);
  Serial.print(" ");
  Serial.print(packet.payload.raw[4], HEX);
  Serial.print(" ");
  Serial.print(packet.payload.raw[5], HEX);
  Serial.print(" ");
  Serial.print(packet.payload.raw[6], HEX);
  Serial.print(" ");
  Serial.println(packet.payload.raw[7], HEX);
  Serial.print("     cs: 0x");
  Serial.println(packet.checkSum, HEX);
  Serial.println("---------------------");
}

UartTask::UartTask() : m_recvLen(0), SerialPi(PI_TX_PIN, PI_RX_PIN) {}

UartTask::~UartTask() {}

void UartTask::setup() {
  Serial.println("setting up uart task...");

  SerialPi.begin(115200, SERIAL_8N1);

  Serial.println("uart task started.");
}

unsigned long UartTask::loop(MicroTasks::WakeReason reason) {

  // Serial.println("uart loop begin.");
#ifdef ENABLE_DEBUG_MESSAGE
  if (reason != WakeReason_Scheduled) {
    Serial.print("UartTask woke: ");
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
#ifdef ENABLE_DEBUG_MESSAGE
      Serial.print("Got message ");
#endif
      switch (msg->id()) {
      case EncoderPositionMessage::ID: {
        EncoderPositionMessage *_msg = (EncoderPositionMessage *)msg;
        sendPacket(0x82, _msg->m_left, _msg->m_right);
        break;
      }
      case DistanceMessage::ID: {
        DistanceMessage *_msg = (DistanceMessage *)msg;
        sendPacket(0x83, _msg->m_distances[0], _msg->m_distances[1],
                   _msg->m_distances[2], _msg->m_distances[3]);
        break;
      }
      case BatteryMessage::ID: {
        BatteryMessage *_msg = (BatteryMessage *)msg;
        sendPacket(0x84, (float)_msg->m_mV / 1000.0f, _msg->m_percent);
        break;
      }
      }
      delete msg;
    }
  }

  int available = SerialPi.available();
  while (available > 0) {
    Serial.print("do reading uart from offset ");
    Serial.println(m_recvLen);
    // do reading
    int bytesRead =
        SerialPi.readBytes(m_recvBuf + m_recvLen, UART_PACKET_LEN - m_recvLen);
    m_recvLen += bytesRead;
    available -= bytesRead;

    if (m_recvLen == UART_PACKET_LEN) {
      if (m_recvBuf[0] != PACKET_HEADER) {
        // drop until header was found
        int headerLoc;
        for (headerLoc = 0;
             headerLoc < m_recvLen && m_recvBuf[headerLoc] != PACKET_HEADER;
             ++headerLoc)
          ;
        if (headerLoc < m_recvLen) {
          // found
          int len = m_recvLen - headerLoc - 1;
          for (int i = 0; i < len; ++i) {
            m_recvBuf[i] = m_recvBuf[headerLoc + i];
          }
          m_recvLen = len;
        } else {
          m_recvLen = 0;
        }
        // incomplete packet, resume receiving
        Serial.print("incomplete packet, skipped ");
        Serial.print(UART_PACKET_LEN - m_recvLen);
        Serial.println(" bytes");
        continue;
      }
      // process packet
      UartPacket *packet = (UartPacket *)m_recvBuf;
      m_recvLen = 0;
      // validating checksum
      if (packet->checkSum != calcCheckSum(*packet)) {
        Serial.println("invalid packet cs, resume receiving.");
        debugPrintUartPacket(*packet);
        continue;
      }
      // Serial.println("got packet");
      // debugPrintUartPacket(*packet);
      switch (packet->cmd) {
      case 0x01: {
        // throttle | 0x01     | left motor(int32), right motor(int32). value:
        // -255 ~ 255
        roverTask.send(new ThrottleMessage(ThrottleMessage::TMT_LEFT,
                                           packet->payload.i32Array[0]));
        roverTask.send(new ThrottleMessage(ThrottleMessage::TMT_RIGHT,
                                           packet->payload.i32Array[1]));
        break;
      }
      case 0x02: {
        // cmd_vel | 0x02 | left speed(m / s, float32), right speed(m /
        // s, float32)
        roverTask.send(new LinearSpeedMessage(0, packet->payload.f32Array[0]));
        roverTask.send(new LinearSpeedMessage(1, packet->payload.f32Array[1]));
        break;
      }
      case 0x03: {
        // query ultrasonic | 0x03 | /
        sendPacket(0x83, (int16_t)ultrasonicTask.distance(), 0, 0, 0);
        break;
      }
      case 0x04: {
        // query battery | 0x04 | /
        sendPacket(0x84, batteryTask.batteryVoltage(),
                   batteryTask.batteryVoltage() / 126.0f);
        break;
      }
      case 0x05: {
        // query speed | 0x05 | /
        sendPacket(0x84, roverTask.leftSpeed(), roverTask.rightSpeed());
        break;
      }
      }
    }
  }
  // Serial.println("uart loop end.");
  return 10; // running at 100Hz
}

void UartTask::sendPacket(uint8_t cmd, int32_t d0, int32_t d1) {
  UartPacket packet;
  packet.header = PACKET_HEADER;
  packet.cmd = cmd;
  packet.payload.i32Array[0] = d0;
  packet.payload.i32Array[1] = d1;
  packet.checkSum = calcCheckSum(packet);
  // debugPrintUartPacket(packet);
  SerialPi.write((uint8_t *)&packet, UART_PACKET_LEN);
  // Serial.println("packet sent");
}

void UartTask::sendPacket(uint8_t cmd, float f0, float f1) {
  UartPacket packet;
  packet.header = PACKET_HEADER;
  packet.cmd = cmd;
  packet.payload.f32Array[0] = f0;
  packet.payload.f32Array[1] = f1;
  packet.checkSum = calcCheckSum(packet);
  // debugPrintUartPacket(packet);
  SerialPi.write((uint8_t *)&packet, UART_PACKET_LEN);
  // Serial.println("packet sent");
}

void UartTask::sendPacket(uint8_t cmd, int16_t d0, int16_t d1, int16_t d2,
                          int16_t d3) {
  UartPacket packet;
  packet.header = PACKET_HEADER;
  packet.cmd = cmd;
  packet.payload.i16Array[0] = d0;
  packet.payload.i16Array[1] = d1;
  packet.payload.i16Array[2] = d2;
  packet.payload.i16Array[3] = d3;
  packet.checkSum = calcCheckSum(packet);
  // debugPrintUartPacket(packet);
  SerialPi.write((uint8_t *)&packet, UART_PACKET_LEN);
  // Serial.println("packet sent");
}

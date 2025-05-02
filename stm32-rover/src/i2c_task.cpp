#include "i2c_task.h"
#include "board_def.h"

#include <Arduino.h>
#include <Wire.h>

#include "battery_task.h"
#include "led_task.h"
#include "rover_task.h"
#include "ultrasonic_task.h"

#include "msgs.h"

extern BatteryTask batteryTask;
extern I2cTask i2cTask;
extern LedTask ledTask;
extern RoverTask roverTask;
extern UltrasonicTask ultrasonicTask;

static uint8_t i2cRegister = 0;
static uint8_t i2cReadRegister = 0;

I2cTask::I2cTask() {}

I2cTask::~I2cTask() {}

void I2cTask::setup() {

  Serial.println("setting up i2c task...");

  // fix stm32f103c8t6 i2c issue
  NVIC_SetPriority(TIM1_CC_IRQn, 0xFF);

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onRequest(onI2CRequestEvent);
  Wire.onReceive(onI2CData);

  Serial.println("i2c task started.");
}

unsigned long I2cTask::loop(MicroTasks::WakeReason reason) { return INFINITY; }

// ----------------------------------

void I2cTask::onI2CRequestEvent() {
  // ISR函数
  int32_t value = 0;
  switch (i2cReadRegister) {
  case 0x81:
    // 获取编码器A里程
    value = roverTask.leftEncoder().pos();
    break;
  case 0x82:
    // 获取编码器B里程
    value = roverTask.rightEncoder().pos();
    break;
  case 0x83:
    // 获取超声波读数
    // value = distance;
    break;
  case 0x84:
    // 获取电池电压
    value = batteryTask.batteryVoltage();
    break;
  case 0x85:
    // 获取编码器A(H)B(L)速度
    value = (roverTask.leftSpeed() << 16) + roverTask.rightSpeed();
    break;
  case 0x86:
    // 获取PID激活状态
    value = roverTask.isPidEnabled() ? 1 : 0;
  default:
    break;
  }
  Wire.write((uint8_t *)&value, sizeof(int32_t));
  i2cReadRegister = 0;
}

void I2cTask::onI2CData(int bytesCount) {
  // ISR函数
  while (0 < Wire.available()) // loop through all but the last
  {
    int8_t c = Wire.read(); // receive byte as a character
    if (i2cRegister != 0) {
      switch (i2cRegister) {
      case 0x01:
        // 设置左电机油门
        roverTask.send(new ThrottleMessage(ThrottleMessage::TMT_LEFT, c));
        break;
      case 0x02:
        // 设置右电机油门
        roverTask.send(new ThrottleMessage(ThrottleMessage::TMT_RIGHT, c));
        break;
      case 0x03: {
        // 重置编码器里程
        roverTask.send(new ResetEncoderPositionMessage());
        break;
      }
      case 0x04: {
        // 设置左轮线速度
        roverTask.send(new LinearSpeedMessage(0, c * 10.0f));
        break;
      }
      case 0x05: {
        // 设置右轮线速度
        roverTask.send(new LinearSpeedMessage(1, c * 10.0f));
        break;
      }
      case 0x06: {
        // 设置PID
        roverTask.enablePID(c > 0);
        break;
      }
      default:
        break;
      }
      i2cRegister = 0;
    } else {
      if (c >= 0x80 || c < 0)
        i2cReadRegister = c;
      else
        i2cRegister = c;
    }
  }
}

#include <Arduino.h>

#include "MicroTasks.h"
#include "battery_task.h"
#include "i2c_task.h"
#include "led_task.h"
#include "mpu6050_task.h"
#include "rover_task.h"
#include "uart_task.h"
#include "ultrasonic_task.h"

BatteryTask batteryTask;
I2cTask i2cTask;
UartTask uartTask;
UartRecvTask uartRecvTask(uartTask.getSerialPi());
LedTask ledTask(false);
RoverTask roverTask;
UltrasonicTask ultrasonicTask;
// Mpu6050Task imuTask;

void setup() {
  Serial.begin(9600);
  Serial.println("\ninitializing tasks");

  MicroTask.startTask(roverTask);
  MicroTask.startTask(ledTask);
  MicroTask.startTask(batteryTask);
  // MicroTask.startTask(i2cTask);
  MicroTask.startTask(uartTask);
  delay(1000);
  MicroTask.startTask(uartRecvTask);
  MicroTask.startTask(ultrasonicTask);
  // MicroTask.startTask(imuTask);

  Serial.println("start");
}

void loop() { MicroTask.update(); }
#include <Arduino.h>

#include "MicroTasks.h"
#include "battery_task.h"
#include "i2c_task.h"
#include "led_task.h"
#include "rover_task.h"
#include "uart_task.h"
#include "ultrasonic_task.h"

BatteryTask batteryTask;
I2cTask i2cTask;
UartTask uartTask;
LedTask ledTask(false);
RoverTask roverTask;
UltrasonicTask ultrasonicTask;

void setup() {
  Serial.begin(9600);
  Serial.println("\ninitializing tasks");

  MicroTask.startTask(roverTask);
  MicroTask.startTask(ledTask);
  MicroTask.startTask(batteryTask);
  // MicroTask.startTask(i2cTask);
  MicroTask.startTask(uartTask);
  MicroTask.startTask(ultrasonicTask);

  Serial.println("start");
}

void loop() { MicroTask.update(); }
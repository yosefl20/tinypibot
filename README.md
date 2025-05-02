# 手搓廉价二轮差速ROS底盘

## 硬件组成

电机：2xTT直流电机（减速比1:48），霍尔编码器（11线）。
电机驱动：TB6612
IMU：MPU6050
超声波测距：HC-SR04+
激光雷达：SC MINI
MCU：STM32F1C8T8
上位机：Raspberry PI 3B+
摄像头：树莓派CSI摄像头
电源：3S 18650, 1xDCDC模块

## 工程说明

### stm32-rover

底盘控制器固件源码，使用`platforio+ststm32`开发。

构建：

```
pio run -e bluepill_f103c8_dev
```

### catkin_ws

工作电脑跑的工作空间，包含：

模块名 | 功能
----- | -----
tinybot_slam | slam功能包，包含gmapping和karto。cartographer还在增加中
tinybot_rviz | 看可视化内容用

### pi_catkin_ws

在底盘上位机跑的工作空间，包含：

模块名 | 功能
----- | -----
ds4_driver | PS4手柄驱动
i2c_device_ros | I2C驱动
i2c_rover | 底盘行驶控制模块
launch | launch文件所在位置，没做成pkg，不规范了点
mpu6050_driver | mpu6050驱动
robot_pose_ekf | robot pose ekf功能包
sc_mini | 单线激光雷达驱动
teleop_twist_keyboard_cpp | 键盘操作功能包
tinypibot_urdf | urdf包
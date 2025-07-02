#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdlib.h>
#include <string>
#include <thread>
#include <unistd.h>

#include "../serial_port.h"
#include "tinybot_base/tinybot_uart_protocol.h"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class TinybotBase : public rclcpp::Node {
public:
  TinybotBase(const std::string &nodeName);
  ~TinybotBase();

  const std::string &portDev() const { return m_serialDev; }

protected:
  uint8_t calcCheckSum(const UartPacket &packet);
  void debugPrintPacket(const UartPacket &packet);

protected:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timer_100ms_callback();
  void readSerialData();
  bool imu_calibration();

private:
  rclcpp::Time m_currentTime;

  // params
  std::string m_serialDev;
  float m_correctFactorVx;
  float m_correctFactorVth;
  float m_bodyWidth;
  float m_wheelSize;
  float m_wheelDistance;

  // components
  std::shared_ptr<std::thread> m_readSerialThread;
  rclcpp::TimerBase::SharedPtr m_timer100ms;
  bool m_autoStopOn = true;
  bool m_useImu = false;
  bool m_pubOdom = false;
  unsigned int m_autoStopCount = 0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odomPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPublisher;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      m_cmdVelSubscription;
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;

  SerialPort m_serial;
  // cached data
};
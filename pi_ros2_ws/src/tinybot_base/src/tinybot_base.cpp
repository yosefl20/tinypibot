#include "tinybot_base/tinybot_base.h"

static const float TICKS_PER_CYCLE = 528.0f;

uint8_t TinybotBase::calcCheckSum(const UartPacket &packet) {
  int32_t val = packet.header + packet.cmd + packet.payload.raw[0] +
                packet.payload.raw[1] + packet.payload.raw[2] +
                packet.payload.raw[3] + packet.payload.raw[4] +
                packet.payload.raw[5] + packet.payload.raw[6] +
                packet.payload.raw[7];
  return val & 0xff;
}

void TinybotBase::debugPrintPacket(const UartPacket &packet) {
  RCLCPP_INFO(this->get_logger(), "UART PACKET: \n\
     header: 0x%02X\n\
        cmd: 0x%02X\n\
    payload: %02X %02X %02X %02X %02X %02X %02X %02X\n\
   checksum: 0x%02X",
              packet.header, packet.cmd, packet.payload.raw[0],
              packet.payload.raw[1], packet.payload.raw[2],
              packet.payload.raw[3], packet.payload.raw[4],
              packet.payload.raw[5], packet.payload.raw[6],
              packet.payload.raw[7], packet.checkSum);
}

TinybotBase::TinybotBase(const std::string &nodeName) : Node(nodeName) {
  // 加载参数
  std::string port_name = "ttyS5";
  this->declare_parameter("port_name", "ttyS5"); // 声明及获取串口号参数
  this->get_parameter_or<std::string>("port_name", port_name, "ttyS5");
  this->declare_parameter("correct_factor_vx", 1.0); // 声明及获取线速度校正参数
  this->get_parameter_or<float>("correct_factor_vx", m_correctFactorVx, 1.0);
  this->declare_parameter("correct_factor_vth",
                          1.0); // 声明及获取角速度校正参数
  this->get_parameter_or<float>("correct_factor_vth", m_correctFactorVth, 1.0);
  this->declare_parameter("auto_stop_on",
                          true); // 声明及获取自动停车功能的开关值
  this->get_parameter_or<bool>("auto_stop_on", m_autoStopOn, true);
  this->declare_parameter("use_imu", false); // 声明是否使用imu
  this->get_parameter_or<bool>("use_imu", m_useImu, false);
  this->declare_parameter("pub_odom", false); // 声明是否发布odom的tf
  this->get_parameter_or<bool>("pub_odom", m_pubOdom, false);
  this->declare_parameter("wheel_size",
                          0.068f); // 声明及获取车轮直径参数
  this->get_parameter_or<float>("wheel_size", m_wheelSize, 0.068f);
  this->declare_parameter("wheel_distance",
                          0.114f); // 声明及获取车轮间距参数
  this->get_parameter_or<float>("wheel_distance", m_wheelDistance, 0.114f);
  this->declare_parameter("body_width",
                          0.14f); // 声明及获取车宽
  this->get_parameter_or<float>("body_width", m_bodyWidth, 0.14f);

  // 打印加载的参数值
  RCLCPP_INFO(this->get_logger(), "Loading parameters: \n \
            - port name: %s\n \
            - correct factor vx: %0.4f\n \
            - correct factor vth: %0.4f\n \
            - auto stop on: %d\n \
            - use imu: %d\n \
            - pub odom: %d\n",
              port_name.c_str(), m_correctFactorVx, m_correctFactorVth,
              m_autoStopOn, m_useImu, m_pubOdom);

  m_serialDev = "/dev/" + port_name;
  m_odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  m_cmdVelSubscription = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&TinybotBase::cmd_vel_callback, this, std::placeholders::_1));

  // 创建TF广播器
  m_tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 控制器与扩展驱动板卡的串口配置与通信
  if (m_serial.open(m_serialDev.c_str(), 115200) < 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "tinyport can not open serial port,Please check the serial "
                 "port cable! "); // 如果开启串口失败，打印错误信息
  } else {
    RCLCPP_INFO(this->get_logger(),
                "tinyport serial port opened"); // 串口开启成功提示

    // 启动一个新线程读取并处理串口数据
    m_readSerialThread = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&TinybotBase::readSerialData, this)));
  }

  if (m_useImu) {
    // 创建IMU的话题发布者
    m_imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

    // IMU初始化标定
    if (imu_calibration()) {
      usleep(500000); // 确保标定完成
      RCLCPP_INFO(this->get_logger(), "IMU calibration ok.");
    }
  }

  // 设置LED灯的初始状态
  // robot_status_.led_on = true;

  // 启动一个100ms的定时器，处理订阅者之外的其他信息
  m_timer100ms = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TinybotBase::timer_100ms_callback, this));

  RCLCPP_INFO(this->get_logger(), "Tinybot Start, enjoy it.");
}

TinybotBase::~TinybotBase() {}

void TinybotBase::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {}

void TinybotBase::timer_100ms_callback() {}

void TinybotBase::readSerialData() {
  UartPacket packet;
  int bytesRead = 0;
  int tlen;
  rclcpp::Time last_time = this->get_clock()->now();
  double x = 0, y = 0, th = 0;

  while (rclcpp::ok()) {
    if (bytesRead == 0) {
      // try reading header
      tlen = m_serial.read((uint8_t *)&packet, 1);
      if (tlen == 1 && packet.header != PACKET_HEADER) {
        RCLCPP_WARN(this->get_logger(), "incomplete packet, resume receiving.");
        continue;
      }
      bytesRead = tlen;
    }
    tlen = m_serial.read((uint8_t *)&packet + bytesRead,
                         UART_PACKET_LEN - bytesRead);
    if (tlen <= 0)
      continue;

    bytesRead += tlen;
    if (bytesRead == UART_PACKET_LEN) {
      bytesRead = 0;
      // debugPrintPacket(packet);
      if (packet.checkSum != calcCheckSum(packet)) {
        RCLCPP_WARN(this->get_logger(),
                    "invalid packet, expected cs = %d, packet.cs = %d",
                    calcCheckSum(packet), packet.checkSum);
        continue;
      }
      const rclcpp::Time current_time = this->get_clock()->now();
      const double dtime = (current_time - last_time).seconds();
      switch (packet.cmd) {
      case 0x82: {
        double dleft =
            packet.payload.i32Array[0] * M_PI * m_wheelSize /
            TICKS_PER_CYCLE; // 计算左轮一周期内的运动路程，一圈为11*48个脉冲值
        double dright = (-packet.payload.i32Array[1]) * M_PI * m_wheelSize /
                        TICKS_PER_CYCLE; // 计算右轮一周期内的运动路程
        double vth = (dright - dleft) / m_bodyWidth;
        double vxy = (dleft + dright) / 2.0;
        x += vxy * cos(th + vth * 0.5f);
        y += vxy * sin(th + vth * 0.5f);
        th += vth;
        // publish_odom(odomPub, tfbc, x, y, th, vxy, 0, vth, current_time);
        break;
      }
      case 0x83: {
        sensor_msgs::msg::Range range_msg;
        range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        range_msg.header.frame_id = "camera";
        range_msg.header.stamp = current_time;
        range_msg.field_of_view = 1.04; // fake
        range_msg.min_range = 0.0;
        range_msg.max_range = 2.56;
        range_msg.range =
            (float)packet.payload.i16Array[0] / 100.0f; // convert cm to m
        // rangePub.publish(range_msg);
        break;
      }
      case 0x84: {
        RCLCPP_INFO(this->get_logger(), "battery voltage: %f V",
                    packet.payload.f32Array[0] * 1200.0f / 1000.0f);
        break;
      }
      case 0x85: {
        // encoder & odom
        if (dtime > 1e-5) {
          double dleft =
              packet.payload.f32Array[0] * dtime * M_PI * m_wheelSize /
              TICKS_PER_CYCLE; // 计算左轮一周期内的运动路程，一圈为11*48个脉冲值
          double dright = packet.payload.f32Array[1] * dtime * M_PI *
                          m_wheelSize /
                          TICKS_PER_CYCLE; // 计算右轮一周期内的运动路程

          double vth = (dright - dleft) / m_bodyWidth;
          double vxy = (dleft + dright) / 2.0;
          x += vxy * cos(th + vth * 0.5f);
          y += vxy * sin(th + vth * 0.5f);
          th += vth;
          // 校正姿态角度，让机器人处于-180~180度之间
          if (th > M_PI)
            th -= M_PI * 2;
          else if (th < (-M_PI))
            th += M_PI * 2;

          // publish_odom(odomPub, tfbc, x, y, th, vxy, 0, vth, current_time);
        }
        break;
      }
      case 0x86: {
        // imu ypr
        break;
      }
      case 0x87: {
        // imu accel
        break;
      }
      case 0x88: {
        // imu gyro
        break;
      }
      }
      last_time = current_time;
    }
  }
}

bool TinybotBase::imu_calibration() { return true; }
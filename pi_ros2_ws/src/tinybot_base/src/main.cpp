#include "serial_port.h"
#include "tinybot_base/tinybot_base.h"

std::string g_chasisDev("/dev/ttyS5");

void sigintHandler(int) {

  printf("Tinybot shutdown...\n");

  SerialPort serial;
  if (serial.open(g_chasisDev.c_str(), 115200)) {
    // 停止底盘
    // 关闭串口
    serial.close();
  }

  // 关闭ROS2接口，清除资源
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
  // 初始化ROS节点
  rclcpp::init(argc, argv);

  // 创建信号处理函数
  signal(SIGINT, sigintHandler);

  // 创建机器人底盘类，通过spin不断查询订阅话题
  auto node = std::make_shared<TinybotBase>("tinybot_base");
  g_chasisDev = std::string("/dev/") + node->portDev();
  rclcpp::spin(node);

  // 关闭ROS2接口，清除资源
  rclcpp::shutdown();

  return 0;
}
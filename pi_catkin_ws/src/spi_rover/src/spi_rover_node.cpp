#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <spidev_lib++.h>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

SPI *mySPI = NULL;
uint8_t rx_buffer[32];

ros::Publisher *g_odomPub = NULL;

void inputCallback(const std_msgs::String::ConstPtr &msg) {

  ROS_INFO("I heard: [%s]", msg->data.c_str());
  if (mySPI == NULL) {
    ROS_ERROR("SPI not ready");
    return;
  }

  uint8_t tx_buffer[2];
  if (msg->data == "FW") {
    tx_buffer[0] = 0x01;
    tx_buffer[1] = 75;
    mySPI->write(tx_buffer, 2);
  } else if (msg->data == "BW") {
    tx_buffer[0] = 0x01;
    tx_buffer[1] = -75;
    mySPI->write(tx_buffer, 2);
  } else if (msg->data == "LF") {
    tx_buffer[0] = 0x02;
    tx_buffer[1] = 75;
    mySPI->write(tx_buffer, 2);
  } else if (msg->data == "RT") {
    tx_buffer[0] = 0x02;
    tx_buffer[1] = -75;
    mySPI->write(tx_buffer, 2);
  } else if (msg->data == "ST") {
    tx_buffer[0] = 0x01;
    tx_buffer[1] = 0;
    mySPI->write(tx_buffer, 2);
  }
}

void joyInputCallback(const sensor_msgs::Joy::ConstPtr &msg) {

  static float x = 0, y = 0;
  const float maxTorch = 100.0f;

  if (mySPI == NULL) {
    ROS_ERROR("SPI not ready");
    return;
  }

  // ROS_INFO("joy msg: axes [%f, %f]", msg->axes[0], msg->axes[1]);
  uint8_t tx_buffer[2];
  if (fabs(msg->axes[0] - x) > 0.01f || fabs(msg->axes[1] - y) > 0.01f) {
    if (fabs(msg->axes[0]) < 1e-5 && fabs(msg->axes[1]) < 1e-5) {
      // stop
      tx_buffer[0] = 0x01;
      tx_buffer[1] = 0;
      mySPI->write(tx_buffer, 2);
    } else if (fabs(msg->axes[0]) > 0.5f) {
      tx_buffer[0] = 0x02;
      tx_buffer[1] = (int8_t)(msg->axes[0] * maxTorch);
      mySPI->write(tx_buffer, 2);
    } else if (fabs(msg->axes[1]) > 1e-5) {
      tx_buffer[0] = 0x01;
      tx_buffer[1] = (int8_t)(msg->axes[1] * maxTorch);
      mySPI->write(tx_buffer, 2);
    }
    x = msg->axes[0];
    y = msg->axes[1];
  }
}

void encoderCallback(const ros::TimerEvent &) {

  if (mySPI == NULL) {
    ROS_ERROR("SPI not ready");
    return;
  }

  // 电机减速比：1:48
  // 编码器线数：13
  // 一圈信号数：624
  static ros::Time last_time = ros::Time::now();
  const ros::Time current_time = ros::Time::now();
  const float wheelWidth = 0.14f;
  const float wheelSize = 0.065f;
  static double x = 0, y = 0, th = 0;
  const double dtime = current_time.toSec() - last_time.toSec();

  if (dtime < 1e-5)
    return;

  ROS_INFO("encoderCallback triggered");
  int32_t left_pulse, right_pulse;
  uint8_t tx_buffer = 0x81;
  mySPI->write(&tx_buffer, 1);
  mySPI->read((uint8_t *)&left_pulse, 4);
  tx_buffer = 0x82;
  mySPI->write(&tx_buffer, 1);
  mySPI->read((uint8_t *)&right_pulse, 4);

  // process
  left_pulse = -left_pulse;
  ROS_INFO("got %d / %d", left_pulse, right_pulse);

  double dleft = left_pulse * M_PI * wheelSize /
                 624.0; // 计算左轮一周期内的运动路程，一圈为624个脉冲值
  double dright =
      right_pulse * M_PI * wheelSize / 624.0; // 计算右轮一周期内的运动路程
  double dth = (dleft - dright) / wheelWidth;
  double vx = (dleft + dright) / 2.0 / dtime;
  double vy = 0.0;
  double vth = dth / dtime;
  x += vx * cos(th + vth / 2.0) * dtime;
  y += vx * sin(th + vth / 2.0) * dtime;
  th += dth;

  nav_msgs::Odometry odom;
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, th);

  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link"; // set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf2::toMsg(orientation); // set the velocity
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  // SymmetricMatrix measNoiseOdom_Cov(6); measNoiseOdom_Cov = 0;
  odom.pose.covariance[0] = pow(10, -2); // = 0.01221 meters / sec
  odom.pose.covariance[7] = pow(10, -2); // = 0.01221 meters / sec
  odom.pose.covariance[14] = pow(10, 6); // = 0.01221 meters / sec

  odom.pose.covariance[21] = pow(10, 6); // = 0.41 degrees / sec
  odom.pose.covariance[28] = pow(10, 6); // = 0.41 degrees / sec
  odom.pose.covariance[35] = pow(
      10, 6); // 0.2;// pow(0.1,2) = 0.41 degrees / sec    //publish the message

  if (g_odomPub)
    g_odomPub->publish(odom);
  last_time = current_time;
}

int main(int argc, char **argv) {

  // spi 设置
  spi_config_t spi_config;
  spi_config.mode = 0;
  spi_config.speed = 1000000;
  spi_config.delay = 0;
  spi_config.bits_per_word = 8;

  mySPI = new SPI("/dev/spidev0.0", &spi_config);

  if (!mySPI->begin()) {
    ROS_INFO("cannot begin spi!");
    delete mySPI;
    return 1;
  }

  ROS_INFO("initializing node...");

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Publisher odom_pub =
      n.advertise<nav_msgs::Odometry>("tinypibot/odom", 50);
  g_odomPub = &odom_pub;

  ROS_INFO("setup encoder timer...");
  ros::Timer timer1 = n.createTimer(ros::Duration(0.5), encoderCallback);

  ROS_INFO("subscribing topic /controller_input...");
  ros::Subscriber sub = n.subscribe("controller_input", 100, inputCallback);
  ROS_INFO("subscribing topic /joy...");
  ros::Subscriber sub2 = n.subscribe("joy", 100, joyInputCallback);

  ROS_INFO("spin.");
  ros::spin();

  delete mySPI;
  return 0;
}

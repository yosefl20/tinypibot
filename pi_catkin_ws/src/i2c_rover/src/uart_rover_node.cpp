#define MPU6050_INCLUDE_DMP_MOTIONAPPS20 1

#include "geometry_msgs/Twist.h"
#include "mpu6050/MPU6050.h"
#include "mpu6050/glove.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Range.h"
#include "serial_port.h"
#include "std_msgs/String.h"
#include <cmath>
#include <i2cdev/I2Cdev.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <span>
#include <sstream>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

#define ROVER_UART_FILE "/dev/ttyAMA2"
#define PACKET_HEADER 0xE7

static int8_t g_leftThrottle = 0, g_rightThrottle = 0;
static int8_t g_leftTicksPerSec = 0, g_rightTicksPerSec = 0;
static bool g_throttleChanged = false;
static const float gravity_value = 9.81;
static const float deg_to_rad_factor = M_PI / 180.0;
static const float TICKS_PER_CYCLE = 528.0f;
static SerialPort g_roverSerial;

#define UART_PACKET_LEN 12

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

static void sigintHandler(int sig) {

  printf("Tinybot shutdown...\n");

  if (!g_roverSerial.isOpen()) {
    g_roverSerial.open(ROVER_UART_FILE, 115200);
  }
  // stop wheels
  UartPacket packet;
  packet.header = PACKET_HEADER;
  packet.cmd = 0x02;
  packet.payload.f32Array[0] = 0;
  packet.payload.f32Array[1] = 0;
  packet.checkSum = calcCheckSum(packet);
  g_roverSerial.writeAll((uint8_t *)&packet, UART_PACKET_LEN);
  g_roverSerial.close();
  ros::shutdown();
}

static void debugPrintPacket(const UartPacket &packet) {
  ROS_INFO("UART PACKET: \n\
     header: 0x%02X\n\
        cmd: 0x%02X\n\
    payload: %02X %02X %02X %02X %02X %02X %02X %02X\n\
   checksum: 0x%02X",
           packet.header, packet.cmd, packet.payload.raw[0],
           packet.payload.raw[1], packet.payload.raw[2], packet.payload.raw[3],
           packet.payload.raw[4], packet.payload.raw[5], packet.payload.raw[6],
           packet.payload.raw[7], packet.checkSum);
}

void inputCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  if (msg->data == "FW") {
    g_leftThrottle = 75;
    g_rightThrottle = 75;
  } else if (msg->data == "BW") {
    g_leftThrottle = -75;
    g_rightThrottle = -75;
  } else if (msg->data == "LF") {
    g_leftThrottle = -75;
    g_rightThrottle = 75;
  } else if (msg->data == "RT") {
    g_leftThrottle = 75;
    g_rightThrottle = -75;
  } else if (msg->data == "ST") {
    g_leftThrottle = 0;
    g_rightThrottle = 0;
  }
}

void cmdVelCallback(const geometry_msgs::Twist &msg) {

  const float maxThrottle = 64.0f;
  // ROS_INFO("cmd_vel msg: linear [%f, %f, %f], angular [%f, %f, %f]",
  //          msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x,
  //          msg.angular.y, msg.angular.z);
  bool usePID = true;

  if (usePID) {
    // 二轮差速运动学逆解
    const float wheelSize = 0.068f; // 0.068m
    const float wheelDistance = 0.114f;
    float out_wheel1_speed = msg.linear.x - msg.angular.z * wheelDistance / 2.0;
    float out_wheel2_speed = msg.linear.x + msg.angular.z * wheelDistance / 2.0;

    // 发送指令
    UartPacket packet;
    packet.header = PACKET_HEADER;
    packet.cmd = 0x02;

    // t0
    packet.payload.f32Array[0] =
        out_wheel1_speed * TICKS_PER_CYCLE / (M_PI * wheelSize);
    // * 0.702f); // 把 m/s 转为 ticks/s, 0.702=左右轮差速系数
    packet.payload.f32Array[1] = out_wheel2_speed * TICKS_PER_CYCLE /
                                 (M_PI * wheelSize); // 把 m/s 转为 ticks/s
    // ROS_INFO("t0: %f ticks/s", packet.payload.f32Array[0]);
    // ROS_INFO("t1: %f ticks/s", packet.payload.f32Array[1]);

    packet.checkSum = calcCheckSum(packet);
    // debugPrintPacket(packet);
    g_roverSerial.writeAll((uint8_t *)&packet, UART_PACKET_LEN);
  } else {
    int8_t left = (int8_t)(maxThrottle * msg.linear.x -
                           maxThrottle * msg.angular.z * 0.5f);
    int8_t right = (int8_t)(maxThrottle * msg.linear.x +
                            maxThrottle * msg.angular.z * 0.5f);

    if (left != g_leftThrottle) {
      g_leftThrottle = left;
      g_throttleChanged = true;
      ROS_INFO("left throttle: %d", g_leftThrottle);
    }
    if (right != g_rightThrottle) {
      g_rightThrottle = right;
      g_throttleChanged = true;
      ROS_INFO("right throttle: %d", g_rightThrottle);
    }
  }
}

void publish_odom(ros::Publisher &odomPub, tf2_ros::TransformBroadcaster &tfbc,
                  float x, float y, float th, float vx, float vy, float vth,
                  const ros::Time &current_time = ros::Time::now()) {
  nav_msgs::Odometry odom;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  // first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // let ekf send the transform
  // tfbc.sendTransform(odom_trans);

  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint"; // set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  odom.twist.covariance[0] = 1e-9;
  odom.twist.covariance[7] = 1e-3;
  odom.twist.covariance[8] = 1e-9;
  odom.twist.covariance[14] = 1e6;
  odom.twist.covariance[21] = 1e6;
  odom.twist.covariance[28] = 1e6;
  odom.twist.covariance[35] = 1e-9;

  // SymmetricMatrix measNoiseOdom_Cov(6); measNoiseOdom_Cov = 0;
  odom.pose.covariance[0] = pow(10, -2); // = 0.01221 meters / sec
  odom.pose.covariance[7] = pow(10, -2); // = 0.01221 meters / sec
  odom.pose.covariance[14] = pow(10, 6); // = 0.01221 meters / sec

  odom.pose.covariance[21] = pow(10, 6); // = 0.41 degrees / sec
  odom.pose.covariance[28] = pow(10, 6); // = 0.41 degrees / sec
  odom.pose.covariance[35] =
      pow(10,
          6); // 0.2;// pow(0.1,2) = 0.41 degrees / sec    //publish the message

  odomPub.publish(odom);
}

void updateChasis(ros::Publisher &odomPub, ros::Publisher &rangePub,
                  tf2_ros::TransformBroadcaster &tfbc) {
  // 电机减速比：1:48
  // 编码器线数：11
  // 一圈信号数：528
  static ros::Time last_time = ros::Time::now();
  const ros::Time current_time = ros::Time::now();
  const float bodyWidth = 0.14f;
  const float wheelSize = 0.068f;
  static double x = 0, y = 0, th = 0;
  const double dtime = current_time.toSec() - last_time.toSec();

  // ROS_INFO("updateChasis triggered");

  if (g_throttleChanged) {
    ROS_INFO("updateChasis updating throttle");
    UartPacket packet;
    packet.header = PACKET_HEADER;
    packet.cmd = 0x01;
    packet.payload.i32Array[0] = g_leftThrottle;
    packet.payload.i32Array[1] = g_rightThrottle;
    packet.checkSum = calcCheckSum(packet);
    g_roverSerial.writeAll((uint8_t *)&packet, UART_PACKET_LEN);
    g_throttleChanged = false;
  } else {
    // reading from serial
    static UartPacket packet;
    static int bytesRead = 0;
    int tlen;
    while (ros::ok()) {
      if (bytesRead == 0) {
        // try reading header
        tlen = g_roverSerial.read((uint8_t *)&packet, 1);
        if (tlen == 1 && packet.header != PACKET_HEADER) {
          ROS_WARN("incomplete packet, resume receiving.");
          break;
        }
        bytesRead = tlen;
      }
      tlen = g_roverSerial.read((uint8_t *)&packet + bytesRead,
                                UART_PACKET_LEN - bytesRead);
      if (tlen <= 0)
        break;

      bytesRead += tlen;
      if (bytesRead == UART_PACKET_LEN) {
        bytesRead = 0;
        // debugPrintPacket(packet);
        if (packet.checkSum != calcCheckSum(packet)) {
          ROS_WARN("invalid packet, expected cs = %d, packet.cs = %d",
                   calcCheckSum(packet), packet.checkSum);
          continue;
        }
        switch (packet.cmd) {
        case 0x82: {
          double dleft =
              packet.payload.i32Array[0] * M_PI * wheelSize /
              TICKS_PER_CYCLE; // 计算左轮一周期内的运动路程，一圈为11*48个脉冲值
          double dright = (-packet.payload.i32Array[1]) * M_PI * wheelSize /
                          TICKS_PER_CYCLE; // 计算右轮一周期内的运动路程

          double vth = (dright - dleft) / bodyWidth;
          double vxy = (dleft + dright) / 2.0;
          x += vxy * cos(th + vth * 0.5f);
          y += vxy * sin(th + vth * 0.5f);
          th += vth;
          publish_odom(odomPub, tfbc, x, y, th, vxy, 0, vth, current_time);
          break;
        }
        case 0x83: {
          sensor_msgs::Range range_msg;
          range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
          range_msg.header.frame_id = "camera";
          range_msg.header.stamp = current_time;
          range_msg.field_of_view = 1.04; // fake
          range_msg.min_range = 0.0;
          range_msg.max_range = 2.56;
          range_msg.range =
              (float)packet.payload.i16Array[0] / 100.0f; // convert cm to m
          rangePub.publish(range_msg);
          break;
        }
        case 0x84: {
          ROS_INFO("battery voltage: %f V",
                   packet.payload.f32Array[0] * 1200.0f / 1000.0f);
          break;
        }
        case 0x85: {
          // encoder & odom
          if (dtime > 1e-5) {
            double dleft =
                packet.payload.f32Array[0] * dtime * M_PI * wheelSize /
                TICKS_PER_CYCLE; // 计算左轮一周期内的运动路程，一圈为11*48个脉冲值
            double dright = packet.payload.f32Array[1] * dtime * M_PI *
                            wheelSize /
                            TICKS_PER_CYCLE; // 计算右轮一周期内的运动路程

            double vth = (dright - dleft) / bodyWidth;
            double vxy = (dleft + dright) / 2.0;
            x += vxy * cos(th + vth * 0.5f);
            y += vxy * sin(th + vth * 0.5f);
            th += vth;
            // 校正姿态角度，让机器人处于-180~180度之间
            if (th > M_PI)
              th -= M_PI * 2;
            else if (th < (-M_PI))
              th += M_PI * 2;

            publish_odom(odomPub, tfbc, x, y, th, vxy, 0, vth, current_time);
          }
          break;
        }
        }
      }
    }
  }
  last_time = current_time;
}

int main(int argc, char **argv) {

  ROS_INFO("initializing i2c...");
  I2Cdev::initialize();
  ROS_INFO("enable i2c");
  I2Cdev::enable(true);

  ROS_INFO("initializing uart...");
  g_roverSerial.open(ROVER_UART_FILE, 115200);
  ROS_INFO("uart " ROVER_UART_FILE " opened");

  signal(SIGINT, sigintHandler);

  {
    // stop wheels
    UartPacket packet;
    packet.header = PACKET_HEADER;
    packet.cmd = 0x02;
    packet.payload.f32Array[0] = 0;
    packet.payload.f32Array[1] = 0;
    packet.checkSum = calcCheckSum(packet);
    g_roverSerial.writeAll((uint8_t *)&packet, UART_PACKET_LEN);
  }

  ROS_INFO("initializing node...");

  ros::init(argc, argv, "pi_rover");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Publisher range_pub = n.advertise<sensor_msgs::Range>("ultrasound", 50);
  ros::Publisher mpu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
  tf2_ros::TransformBroadcaster odom_broadcaster;

  publish_odom(odom_pub, odom_broadcaster, 0, 0, 0, 0, 0, 0);

  ROS_INFO("subscribing topic /controller_input...");
  ros::Subscriber sub = n.subscribe("controller_input", 100, inputCallback);
  ROS_INFO("subscribing topic /cmd_vel...");
  ros::Subscriber sub3 = n.subscribe("cmd_vel", 100, cmdVelCallback);

  ROS_INFO("initializing imu...");
  int imu_num = 1;
  uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
  std::string calib_save_path = "./";
  std::string calib_file_name = "imucalib";

  // MPU control/status vars
  uint8_t devStatus; // return status after each device operation (0 =
                     // success, !0 = error)
  MPU6050 theSensor;

  std::cout << "Testing device connections..." << std::endl;
  if (theSensor.testConnection()) {
    std::cout << "MPU6050 connection successful" << std::endl;
  } else {
    std::cout << "MPU6050 connection failed" << std::endl;
    std::cout << "device id:" << (int)theSensor.getDeviceID() << std::endl;
    return -1;
  }
  std::cout << "Initializing I2C devices..." << std::endl;
  theSensor.initialize();

  FILE *pf_calib_file;
  bool recalib_needed = false;
  pf_calib_file = fopen(
      std::string(calib_save_path + calib_file_name + ".bin").c_str(), "rb");
  if (pf_calib_file == NULL) {
    std::cout
        << "The calibration file of imu doesn't exist. recalibration need!"
        << std::endl;
    pf_calib_file = fopen(
        std::string(calib_save_path + calib_file_name + ".bin").c_str(), "wb");
    std::cout << "Please let the imu stand firmly up. imu " << std::endl;
    recalib_needed = true;
  }
  // theSensor.setFullScaleAccelRange(0);
  sleep(1);

  std::cout << "Initializing DMP..." << std::endl;
  devStatus = theSensor.dmpInitialize();

  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    theSensor.CalibrateAccel(recalib_needed, pf_calib_file, 6);
    theSensor.CalibrateGyro(recalib_needed, pf_calib_file, 6);
    fclose(pf_calib_file);
    theSensor.PrintActiveOffsets();
    usleep(2000);
    // turn on the DMP, now that it's ready
    std::cout << "Enabling DMP..." << std::endl;
    theSensor.setDMPEnabled(true);

    mpuIntStatus = theSensor.getIntStatus();
    std::cout << "MPU_INT_STATUS" << (int)mpuIntStatus << std::endl;
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    std::cout << "DMP Initialization failed (code ";
    std::cout << (int)devStatus;
    std::cout << ")" << std::endl;
    std::cout << "1 = initial memory load failed" << std::endl;
    std::cout << "2 = DMP configuration updates failed" << std::endl;
    std::cout << "(if it's going to break, usually the code will be 1)"
              << std::endl;
  }

  float mpuAccelScalar =
      (float)(1 << theSensor.getFullScaleAccelRange()) / 32767.0f;
  int mpuGyroScalar =
      (float)(1 << theSensor.getFullScaleGyroRange()) / 32767.0f;

  ros::Rate loop_rate(25);
  ROS_INFO("spin.");
  while (ros::ok()) {
    updateChasis(odom_pub, range_pub, odom_broadcaster);

    sensor_msgs::Imu imu_data;
    int16_t imu_raw[6];
    theSensor.getMotion6(&imu_raw[0], &imu_raw[1], &imu_raw[2], &imu_raw[3],
                         &imu_raw[4], &imu_raw[5]);

    imu_data.linear_acceleration.x =
        (float)imu_raw[0] * mpuAccelScalar * 2.0f * gravity_value;
    imu_data.linear_acceleration.y =
        (float)imu_raw[1] * mpuAccelScalar * 2.0f * gravity_value;
    imu_data.linear_acceleration.z =
        (float)imu_raw[2] * mpuAccelScalar * -2.0f * gravity_value;

    imu_data.angular_velocity.x =
        (float)imu_raw[3] * mpuGyroScalar * 250.0f * deg_to_rad_factor;
    imu_data.angular_velocity.y =
        (float)imu_raw[4] * mpuGyroScalar * 250.0f * deg_to_rad_factor;
    imu_data.angular_velocity.z =
        (float)imu_raw[5] * mpuGyroScalar * 250.0f * deg_to_rad_factor;

    imu_data.linear_acceleration_covariance = {0.04, 0.00, 0.00, 0.00, 0.04,
                                               0.00, 0.00, 0.00, 0.04};

    imu_data.angular_velocity_covariance = {0.02, 0.00, 0.00, 0.00, 0.02,
                                            0.00, 0.00, 0.00, 0.02};

    imu_data.orientation_covariance = {0.0025, 0.0000, 0.0000, 0.0000, 0.0025,
                                       0.0000, 0.0000, 0.0000, 0.0025};

    imu_data.header.frame_id = "base_link";
    imu_data.header.stamp = ros::Time::now();

    mpu_pub.publish(imu_data);

    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
  g_roverSerial.close();

  return 0;
}

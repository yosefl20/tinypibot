#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sc_mini/sc_mini.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <string.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "std_srvs/Empty.h"
#include <boost/thread.hpp>

#include "serial_port.h"

#define ROS_N_MUL 2

using namespace std;

static boost::mutex mStausMutex_;
static int laser_fd = -1;
static bool laser_start_flag = true;

unsigned char start_port[10] = {0xA5, 0xF0};
unsigned char end_port[10] = {0xA5, 0xF5};

M1C1_MiniDataStr M1C1_MiniData = {0};

// 重要:
// robot_rotate_compensate_dir只设置两个值+1和-1，意思是做机器人旋转角度补偿时，是按正方向补偿还是按负方向补偿。
//	和实际中机器人底盘顺时针旋转时陀螺仪输出的角度值是增加还是减小有关，所以请按实际来设置是+1还是-1
//	按经验，俯视机器人，机器人顺时针旋转时，陀螺仪数据减小，则该值为+1，否则为-1
//	注意本SDK里计算的的陀螺仪角度数据的单位是度，不是弧度。
const int robot_rotate_compensate_dir = +1;

// 雷达盖子的三个柱子所遮住的起止角度（单位：度），请根据实际安装情况设置
LidarCoverAngleStr LidarCoverAngle[LIDAR_COVER_BAR_NUM] = {
    //{53, 82},
    //{186, 210},
    //{280, 306},
};
// 如果没有雷达盖子，没有屏蔽雷达盖子的需求的话，请将 enable_cover
// 设置为0，否则为1
const int enable_cover = 0;

// 雷达安装到机器人底盘时，两者的零点角度偏差（单位：度），请根据实际安装情况设置
float degree_cali = 0.0;

GyroYawStr GyroYaw = {0};
GyroYawStr GyroYawBak = {0};
float OriginGyroYaw = 0;
float ZoneStartGyro = 0;
int GyroCompensateEnable = 1;

float PI = 3.1415926;
// double angle_yaw_new = 0; //note: 最新的陀螺仪角度值
// double angle_yaw_old = 0; //note: 上一次的陀螺仪角度值
double zone_start_rotate_angle =
    0; // note:
       // 机器人底盘旋转角度补偿起始偏移值：15个角度区域，雷达转到某区域起始的地方时，机器人地盘所旋转过的角度
int Size;
int lidar_start_count;
double clustering_time = 0;

ros::Time lidar_cali_time;

ros::Publisher lidarZero_pub;
ros::Publisher robotErrorTips_pub;

int AllAngleIndex = 0;

/*note:订阅里程计topic进行角度换算，用于机器人旋转过程中对激光雷达角度的补偿*/
// 客户根据自己里程计及陀螺仪的特性处理
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  geometry_msgs::Quaternion quat;
  quat.x = odom->pose.pose.orientation.x;
  quat.y = odom->pose.pose.orientation.y;
  quat.z = odom->pose.pose.orientation.z;
  quat.w = odom->pose.pose.orientation.w;
  tf::Quaternion quat_tf;
  tf::quaternionMsgToTF(quat, quat_tf);
  double roll, pitch, yaw = 0;
  tf::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
  GyroYaw.GyroYaw = (180 / PI) * yaw;
}

namespace sc_m_c {
SCLaser::SCLaser() {
  unsigned int len = 2;
  int result = write_port(start_port, len);
}

SCLaser::~SCLaser() {
  unsigned int len = 2;
  int result = write_port(end_port, len);
}

void SCLaser::start() {
  unsigned int len = 2;
  int result = write_port(start_port, len);
}

void SCLaser::stop() {
  unsigned int len = 2;
  int result = write_port(end_port, len);
}
void PutRemainder2Start(unsigned char *btBuffer, int nStart, int nBufLength) {
  if (nStart > 0) {
    for (int i = nStart; i < nStart + nBufLength; i++) {
      btBuffer[i - nStart] = btBuffer[i];
    }
  }
}

uint8_t MatchZoneNo = 0;
int m_nReceivedLength = 0; // 接收雷达点云数据的接收缓冲区里缓冲的数据长度
int nSeekStart = 0;
char raw_bytes[ANGLE_MAX_NUMBERS]; // 接收雷达点云数据的接收缓冲区
float Angle_in[ANGLE_MAX_NUMBERS];
unsigned char Colour[ANGLE_MAX_NUMBERS];
float FilterRatio =
    2.5; // 滤波功能用到的滤波系数设置为2.5，数值越小滤波效果越强，数值要大于０

/*note: 点云数据简单滤波函数
 *input/output:
 *Scan指针，指向雷达扫描的点云数据，滤波后的结果存在Scan所指向的数组里
 */
void SCLaser::PointCloudFilter(sensor_msgs::LaserScan &Scan) {
  int i0, i1, i2, i3, i4;
  float x0, y0, x1, y1, x2, y2, x3, y3, x4, y4, r1, r2;
  float d01, d12, d23, d34;
  float a, b, Adeta;
  float FilterRatioAdj;
  float AngDiff;
  int DepthState;

  AngDiff = 2 * PI / (Size - 1);
  FilterRatioAdj = FilterRatio * sin(AngDiff);

  for (i0 = 0; i0 < Size; i0++) {
    i1 = i0 + 1;
    i1 = i1 < Size ? i1 : i1 - Size;
    i2 = i0 + 2;
    i2 = i2 < Size ? i2 : i2 - Size;
    i3 = i0 + 3;
    i3 = i3 < Size ? i3 : i3 - Size;
    DepthState = 0;
    DepthState += (Scan.ranges[i0] != 0) * 1;
    DepthState += (Scan.ranges[i1] != 0) * 2;
    DepthState += (Scan.ranges[i2] != 0) * 4;
    DepthState += (Scan.ranges[i3] != 0) * 8;
    if (DepthState == 0x0F) { // 1111
      x0 = Scan.ranges[i0] * cos(Angle_in[i0] * PI / 180);
      y0 = Scan.ranges[i0] * sin(Angle_in[i0] * PI / 180);
      x3 = Scan.ranges[i3] * cos(Angle_in[i3] * PI / 180);
      y3 = Scan.ranges[i3] * sin(Angle_in[i3] * PI / 180);
      x1 = (x3 + 2 * x0) / 3;
      y1 = (y3 + 2 * y0) / 3;
      x2 = (2 * x3 + x0) / 3;
      y2 = (2 * y3 + y0) / 3;
      r1 = sqrt(x1 * x1 + y1 * y1);
      r2 = sqrt(x2 * x2 + y2 * y2);

      a = Scan.ranges[i0];
      b = Scan.ranges[i1];
      Adeta = abs(Angle_in[i0] - Angle_in[i1]) * PI / 180;
      d01 = sqrt(a * a + b * b - 2 * a * b * cos(Adeta));
      a = Scan.ranges[i1];
      b = Scan.ranges[i2];
      Adeta = abs(Angle_in[i1] - Angle_in[i2]) * PI / 180;
      d12 = sqrt(a * a + b * b - 2 * a * b * cos(Adeta));
      a = Scan.ranges[i2];
      b = Scan.ranges[i3];
      Adeta = abs(Angle_in[i2] - Angle_in[i3]) * PI / 180;
      d23 = sqrt(a * a + b * b - 2 * a * b * cos(Adeta));

      if (FilterRatioAdj * Scan.ranges[i1] < d01 &&
          FilterRatioAdj * Scan.ranges[i1] < d12) {
        Scan.ranges[i1] = 0;
      }
      if (FilterRatioAdj * Scan.ranges[i2] < d12 &&
          FilterRatioAdj * Scan.ranges[i2] < d23) {
        Scan.ranges[i2] = 0;
      }
      if (0 == Scan.ranges[i1] || 0 == Scan.ranges[i2]) {
        continue;
      }

      if ((Scan.ranges[i1] > r1 && Scan.ranges[i2] < r2) ||
          (Scan.ranges[i1] < r1 && Scan.ranges[i2] > r2)) {
        if ((d01 > FilterRatioAdj * Scan.ranges[i0] &&
             d12 < FilterRatioAdj * Scan.ranges[i1] &&
             d23 < FilterRatioAdj * Scan.ranges[i2]) ||
            (d01 < FilterRatioAdj * Scan.ranges[i0] &&
             d12 > FilterRatioAdj * Scan.ranges[i1] &&
             d23 < FilterRatioAdj * Scan.ranges[i2]) ||
            (d01 < FilterRatioAdj * Scan.ranges[i0] &&
             d12 < FilterRatioAdj * Scan.ranges[i1] &&
             d23 > FilterRatioAdj * Scan.ranges[i2])) {
        } else {
          Scan.ranges[i1] = r1 + 0.4 * (Scan.ranges[i1] - r1);
          Scan.ranges[i2] = r2 + 0.4 * (Scan.ranges[i2] - r2);
        }
      } else {
      }
    } else if (DepthState == 0x00 || DepthState == 0x08 ||
               DepthState == 0x01) {          // 0000 1000 0001
    } else if ((DepthState & 0x0E) == 0x04) { // 010x
      Scan.ranges[i2] = 0;
    } else if ((DepthState & 0x07) == 0x02) { // x010
      Scan.ranges[i1] = 0;
    } else if ((DepthState & 0x07) == 0x03) { // x011
      a = Scan.ranges[i0];
      b = Scan.ranges[i1];
      if (sqrt(a * a + b * b - 2 * a * b * cos(AngDiff)) >
          FilterRatioAdj * Scan.ranges[i1]) {
        Scan.ranges[i1] = 0;
      }
    } else if (DepthState == 0x0E) { // 1110
      a = Scan.ranges[i1];
      b = Scan.ranges[i2];
      if (sqrt(a * a + b * b - 2 * a * b * cos(AngDiff)) >
          FilterRatioAdj * Scan.ranges[i1]) {
        Scan.ranges[i1] = 0;
      }
    } else if ((DepthState & 0x0E) == 0x0C) { // 110x 0111
      a = Scan.ranges[i2];
      b = Scan.ranges[i3];
      if (sqrt(a * a + b * b - 2 * a * b * cos(AngDiff)) >
          FilterRatioAdj * Scan.ranges[i2]) {
        Scan.ranges[i2] = 0;
      }
    } else if (DepthState == 0x07) { // 0111
      a = Scan.ranges[i1];
      b = Scan.ranges[i2];
      if (sqrt(a * a + b * b - 2 * a * b * cos(AngDiff)) >
          FilterRatioAdj * Scan.ranges[i2]) {
        Scan.ranges[i2] = 0;
      }
    } else if (DepthState == 0x06) { // 0110
      a = Scan.ranges[i1];
      b = Scan.ranges[i2];
      if (sqrt(a * a + b * b - 2 * a * b * cos(AngDiff)) >
          FilterRatioAdj * Scan.ranges[i1]) {
        Scan.ranges[i1] = 0;
        Scan.ranges[i2] = 0;
      }
    } else {
    }
  }

  // 五点拉直

  for (i0 = 0; i0 < Size; i0++) {
    i1 = i0 + 1;
    i1 = i1 < Size ? i1 : i1 - Size;
    i2 = i0 + 2;
    i2 = i2 < Size ? i2 : i2 - Size;
    i3 = i0 + 3;
    i3 = i3 < Size ? i3 : i3 - Size;
    i4 = i0 + 4;
    i4 = i4 < Size ? i4 : i4 - Size;
    if (Scan.ranges[i0] != 0 && Scan.ranges[i1] != 0 && Scan.ranges[i2] != 0 &&
        Scan.ranges[i3] != 0 && Scan.ranges[i4] != 0) {
      a = Scan.ranges[i0];
      b = Scan.ranges[i1];
      Adeta = abs(Angle_in[i0] - Angle_in[i1]) * PI / 180;
      d01 = sqrt(a * a + b * b - 2 * a * b * cos(Adeta));

      a = Scan.ranges[i1];
      b = Scan.ranges[i2];
      Adeta = abs(Angle_in[i1] - Angle_in[i2]) * PI / 180;
      d12 = sqrt(a * a + b * b - 2 * a * b * cos(Adeta));
      a = Scan.ranges[i2];
      b = Scan.ranges[i3];
      Adeta = abs(Angle_in[i2] - Angle_in[i3]) * PI / 180;
      d23 = sqrt(a * a + b * b - 2 * a * b * cos(Adeta));
      a = Scan.ranges[i3];
      b = Scan.ranges[i4];
      Adeta = abs(Angle_in[i3] - Angle_in[i4]) * PI / 180;
      d34 = sqrt(a * a + b * b - 2 * a * b * cos(Adeta));
      if (d01 < FilterRatioAdj * Scan.ranges[i1] &&
          d34 < FilterRatioAdj * Scan.ranges[i3] &&
          (d12 > FilterRatioAdj * Scan.ranges[i2] ||
           d23 > FilterRatioAdj * Scan.ranges[i2])) {
        if ((Scan.ranges[i0] < Scan.ranges[i1] &&
             Scan.ranges[i3] < Scan.ranges[i4]) ||
            (Scan.ranges[i0] > Scan.ranges[i1] &&
             Scan.ranges[i3] > Scan.ranges[i4])) {
          Scan.ranges[i2] = (Scan.ranges[i1] + Scan.ranges[i3]) / 2;
        }
      }
    } else if (Scan.ranges[i0] == 0 && Scan.ranges[i1] != 0 &&
               Scan.ranges[i2] != 0 && Scan.ranges[i3] != 0 &&
               Scan.ranges[i4] != 0) {
      x2 = Scan.ranges[i2] * cos(Angle_in[i2] * PI / 180);
      y2 = Scan.ranges[i2] * sin(Angle_in[i2] * PI / 180);
      x3 = Scan.ranges[i3] * cos(Angle_in[i3] * PI / 180);
      y3 = Scan.ranges[i3] * sin(Angle_in[i3] * PI / 180);
      x1 = 2 * x2 - x3;
      y1 = 2 * y2 - y3;
      Scan.ranges[i1] =
          Scan.ranges[i1] + 0.4 * (sqrt(x1 * x1 + y1 * y1) - Scan.ranges[i1]);
    } else if (Scan.ranges[i0] != 0 && Scan.ranges[i1] != 0 &&
               Scan.ranges[i2] != 0 && Scan.ranges[i3] != 0 &&
               Scan.ranges[i4] == 0) {
      x1 = Scan.ranges[i1] * cos(Angle_in[i1] * PI / 180);
      y1 = Scan.ranges[i1] * sin(Angle_in[i1] * PI / 180);
      x2 = Scan.ranges[i2] * cos(Angle_in[i2] * PI / 180);
      y2 = Scan.ranges[i2] * sin(Angle_in[i2] * PI / 180);
      x3 = 2 * x2 - x1;
      y3 = 2 * y2 - y1;
      Scan.ranges[i3] =
          Scan.ranges[i3] + 0.4 * (sqrt(x3 * x3 + y3 * y3) - Scan.ranges[i3]);
    } else {
    }
  }
}
/*角度插值函数
 * input: scan_in 指针
 * output: scan_out 指针
 * 功能:
 * 将scan_in的点数扩大到400*ROS_N_MUL倍，然后角度插值，得到用于对外发布的scan_out
 */
void SCLaser::angle_insert(sensor_msgs::LaserScan &scan_in,
                           sensor_msgs::LaserScan &scan_out) {
  int temp_i, i, angle, temp_Size;
  float m_fAngle;
  temp_Size =
      180; // note:
           // 雷达转一圈输出的点数按固定的180*ROS_N_MUL点来输出,tof雷达改为720
  // scan_out.ranges.resize(temp_Size*ROS_N_MUL);
  // scan_out.intensities.resize(temp_Size*ROS_N_MUL);
  for (i = 0; i < temp_Size * ROS_N_MUL; i++) {
    scan_out.ranges[i] = std::numeric_limits<float>::
        infinity(); // note:将要发布的激光类距离数据清零，保证做插值时其他数据为零
  }
  for (i = 0; i < Size; i++) {
    temp_i = (int)(Angle_in[i] / (360.0F / (ROS_N_MUL * temp_Size)) +
                   0.5); // note:计算出对应的插值的位置
    temp_i = (temp_i >= temp_Size * ROS_N_MUL) ? 0 : temp_i;
    temp_i = (temp_i < 0) ? 0 : temp_i;
    if (scan_in.ranges[i] == 0)
      scan_out.ranges[temp_i] = std::numeric_limits<float>::infinity();
    else
      scan_out.ranges[temp_i] = scan_in.ranges[i];
    scan_out.intensities[temp_i] =
        scan_out.ranges[temp_i] == 0
            ? 0
            : 127; // note:距离值为0的数据灰度值为0，距离大于0数据灰度值设为127
  }
  scan_out.angle_increment = (2.0 * M_PI / (ROS_N_MUL * temp_Size));
  scan_out.angle_min = 0.0;
  scan_out.angle_max = 2 * M_PI;
  scan_out.range_min = 0.10;
  scan_out.range_max = 10.0; // note: mini_m1c0测量的最远距离是12m
  scan_out.scan_time = clustering_time;
}

/*
 * 接收雷达点云数据的函数
 * input/output: scan指针，雷达扫描的点云数据存在scan所指向的数组里
 * 功能: 1、接收雷达点云数据
 *       2、对激光雷达结构引起的角度误差进行补偿；
 *       3、去除雷达盖子的三根柱子所遮住的数据
 *       4、对激光雷达安装角度进行补偿
 *       5、对机器人底盘旋转角度进行补偿
 */

#define SERIAL_PORT_DATA_BUF_LEN 4096
unsigned char serialport_databuf[SERIAL_PORT_DATA_BUF_LEN];
#define RANGES_TIMP_SIZE 600 // tof雷达改为720

int SCLaser::poll(sensor_msgs::LaserScan &Scan, int fd) {
  int Rcv = 0, cpylen;
  int temp_depth;

  int i;
  float fGyroAngleInc;
  float fStartAngle;
  float fIncAngle;
  float fStopAngle;
  float fTempAngle;
  float angle_c1;
  unsigned char csl, csh;
  int ret = -1;
  GyroYawStr DeltaYaw;

  ros::Time begin_time = ros::Time::now();
  // Scan.ranges.resize(RANGES_TIMP_SIZE);

  Rcv = read_port(serialport_databuf, SERIAL_PORT_DATA_BUF_LEN);
  while (Rcv > 0) {
    if (M1C1_MiniData.BufferLen <
        M1C1_Mini_FRAME_LEN_MAX) // ÊýŸÝ³€¶ÈÌ«¶Ì£¬ŒÌÐøÊÕÊý
    {
      cpylen = Rcv > (M1C1_Mini_FRAME_LEN_MAX - M1C1_MiniData.BufferLen)
                   ? (M1C1_Mini_FRAME_LEN_MAX - M1C1_MiniData.BufferLen)
                   : Rcv;
      memcpy(&M1C1_MiniData.Buffer[M1C1_MiniData.BufferLen], serialport_databuf,
             cpylen);
      Rcv = Rcv - cpylen;
      PutRemainder2Start(serialport_databuf, cpylen, Rcv);
      if (cpylen > 0) {
        M1C1_MiniData.BufferLen += cpylen;
      }
    }

    while (M1C1_MiniData.BufferLen >= M1C1_Mini_FRAME_LEN_MIN) {
      // ros::spinOnce(); //note:触发相应消息订阅函数
      if (M1C1_MiniData.Frame.PHL == 0xAA && M1C1_MiniData.Frame.PHH == 0x55 &&
          M1C1_MiniData.Frame.CT == 0x01 // ÆðÊŒ°ü
          && M1C1_MiniData.Frame.LSN == 1 &&
          (M1C1_MiniData.Frame.FSAL & 0x01) &&
          (M1C1_MiniData.Frame.LSAL == M1C1_MiniData.Frame.FSAL) &&
          (M1C1_MiniData.Frame.LSAH == M1C1_MiniData.Frame.FSAH) &&
          (M1C1_MiniData.Frame.CSL ==
           (M1C1_MiniData.Frame.PHL ^ M1C1_MiniData.Frame.CT ^
            M1C1_MiniData.Frame.FSAL ^ M1C1_MiniData.Frame.LSAL ^
            M1C1_MiniData.Frame.Si[0])) &&
          (M1C1_MiniData.Frame.CSH =
               (M1C1_MiniData.Frame.PHH ^ M1C1_MiniData.Frame.LSN ^
                M1C1_MiniData.Frame.FSAH ^ M1C1_MiniData.Frame.LSAH ^
                M1C1_MiniData.Frame.Si[1])) &&
          AllAngleIndex < ANGLE_MAX_NUMBERS &&
          AllAngleIndex < RANGES_TIMP_SIZE) {
        GyroYawBak = GyroYaw;
        OriginGyroYaw = GyroYaw.GyroYaw;
        ZoneStartGyro = 0;
        temp_depth =
            (M1C1_MiniData.Frame.Si[0] | (M1C1_MiniData.Frame.Si[1] << 8)) / 4;
        Scan.ranges[AllAngleIndex] = temp_depth / 1000.0F;
        Angle_in[AllAngleIndex] =
            (M1C1_MiniData.Frame.FSAL + M1C1_MiniData.Frame.FSAH * 256) / 128.0;
        Size = AllAngleIndex + 1;
        M1C1_MiniData.BufferLen =
            M1C1_MiniData.BufferLen - M1C1_Mini_FRAME_LEN_MIN;
        PutRemainder2Start(M1C1_MiniData.Buffer, M1C1_Mini_FRAME_LEN_MIN,
                           M1C1_MiniData.BufferLen);

        if (AllAngleIndex > ANGLE_MIN_NUMBERS) {
          clustering_time = (ros::Time::now() - begin_time).toSec();
          ret = 0; // ok, receive full frame, exit to draw.
        } else {
          ret = -1; // not full frame.
        }
        AllAngleIndex = 0;
        continue;
      } else if (M1C1_MiniData.Frame.PHL == 0xAA &&
                 M1C1_MiniData.Frame.PHH == 0x55 &&
                 M1C1_MiniData.Frame.CT == 0x00 // ÊýŸÝ°ü
                 && 0 < M1C1_MiniData.Frame.LSN &&
                 M1C1_MiniData.Frame.LSN <= M1C1_Mini_Si_iMAX &&
                 (M1C1_MiniData.Frame.FSAL & 0x01) &&
                 (M1C1_MiniData.Frame.LSAL & 0x01)) {
        if (M1C1_MiniData.BufferLen <
            2 * M1C1_MiniData.Frame.LSN + M1C1_Mini_HEAD_LEN) {
          break;
        }
        csl = M1C1_MiniData.Frame.PHL ^ M1C1_MiniData.Frame.CT ^
              M1C1_MiniData.Frame.FSAL ^ M1C1_MiniData.Frame.LSAL;
        csh = M1C1_MiniData.Frame.PHH ^ M1C1_MiniData.Frame.LSN ^
              M1C1_MiniData.Frame.FSAH ^ M1C1_MiniData.Frame.LSAH;
        for (i = 0; i < M1C1_MiniData.Frame.LSN; i++) {
          csl ^= M1C1_MiniData.Frame.Si[2 * i];
          csh ^= M1C1_MiniData.Frame.Si[2 * i + 1];
        }
        if (M1C1_MiniData.Frame.CSL == csl && M1C1_MiniData.Frame.CSH == csh) {
          // DeltaYaw.timeStamp = GyroYaw.timeStamp - GyroYawBak.timeStamp;
          DeltaYaw.GyroYaw = GyroYaw.GyroYaw - GyroYawBak.GyroYaw;
          GyroYawBak = GyroYaw;
          DeltaYaw.GyroYaw = DeltaYaw.GyroYaw > 360 ? DeltaYaw.GyroYaw - 360
                                                    : DeltaYaw.GyroYaw;
          DeltaYaw.GyroYaw = DeltaYaw.GyroYaw < -360 ? DeltaYaw.GyroYaw + 360
                                                     : DeltaYaw.GyroYaw;
          ZoneStartGyro += DeltaYaw.GyroYaw;

          fStartAngle =
              (M1C1_MiniData.Frame.FSAL + M1C1_MiniData.Frame.FSAH * 256) /
              128.0F;
          fStopAngle =
              (M1C1_MiniData.Frame.LSAL + M1C1_MiniData.Frame.LSAH * 256) /
              128.0F;
          if (fStartAngle < 10) {
            AllAngleIndex = 0;
          }
          if (M1C1_MiniData.Frame.LSN > 1) {
            if (fStartAngle > fStopAngle) {
              fIncAngle = (360 - fStartAngle + fStopAngle) /
                          (M1C1_MiniData.Frame.LSN - 1);

            } else {
              fIncAngle =
                  (fStopAngle - fStartAngle) / (M1C1_MiniData.Frame.LSN - 1);
            }
            fGyroAngleInc = DeltaYaw.GyroYaw / (M1C1_MiniData.Frame.LSN - 1);
          } else {
            fIncAngle = 0;
            fGyroAngleInc = 0;
          }

          for (int j = 0; j < M1C1_MiniData.Frame.LSN &&
                          AllAngleIndex < ANGLE_MAX_NUMBERS &&
                          AllAngleIndex < RANGES_TIMP_SIZE;
               j++) {
            temp_depth = (M1C1_MiniData.Frame.Si[2 * j] |
                          (M1C1_MiniData.Frame.Si[2 * j + 1] << 8)) /
                         4;

            fTempAngle = fStartAngle + j * fIncAngle;

            if (enable_cover) {
              for (i = 0; i < LIDAR_COVER_BAR_NUM; i++) {
                if (LidarCoverAngle[i].f_begin < fTempAngle &&
                    fTempAngle < LidarCoverAngle[i].f_end) {
                  Scan.ranges[AllAngleIndex] = 0;
                }
              }
            }
            /*加入扫地机旋转时变化的角度
             *以及安装激光雷达时偏转的角度
             *  degree_cali是激光雷达安装到机器人地盘上，机器人零角度和激光雷达零角度间的角度差
             */
            // angle_c1 += zone_start_rotate_angle + j*fCompensateStepAngle +
            // degree_cali;
            fTempAngle += ZoneStartGyro;
            fTempAngle -= fGyroAngleInc;
            if (0 <= fTempAngle && fTempAngle <= 360) {
              Scan.ranges[AllAngleIndex] = temp_depth / 1000.0F;
              Angle_in[AllAngleIndex] = 360 - fTempAngle;
            } else {
              Scan.ranges[AllAngleIndex] = 0;
              Angle_in[AllAngleIndex] = 0;
            }
            AllAngleIndex++;
          }
          M1C1_MiniData.BufferLen -=
              (2 * M1C1_MiniData.Frame.LSN + M1C1_Mini_HEAD_LEN);
          PutRemainder2Start(M1C1_MiniData.Buffer,
                             2 * M1C1_MiniData.Frame.LSN + M1C1_Mini_HEAD_LEN,
                             M1C1_MiniData.BufferLen);
        } else {
          for (i = 1; i < M1C1_MiniData.BufferLen - 1; i++) {
            if (M1C1_MiniData.Buffer[i] == 0xAA &&
                M1C1_MiniData.Buffer[i + 1] == 0x55) {
              break;
            }
          }
          M1C1_MiniData.BufferLen = M1C1_MiniData.BufferLen - i;
          PutRemainder2Start(M1C1_MiniData.Buffer, i, M1C1_MiniData.BufferLen);
        }
      } else {
        for (i = 1; i < M1C1_MiniData.BufferLen - 1; i++) {
          if (M1C1_MiniData.Buffer[i] == 0xAA &&
              M1C1_MiniData.Buffer[i + 1] == 0x55) {
            break;
          }
        }
        M1C1_MiniData.BufferLen = M1C1_MiniData.BufferLen - i;
        PutRemainder2Start(M1C1_MiniData.Buffer, i, M1C1_MiniData.BufferLen);
      }
    }
  }
  return ret;
}
} // namespace sc_m_c

bool stop_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  boost::mutex::scoped_lock lock(mStausMutex_);
  if (laser_fd == -1)
    return true;
  ROS_DEBUG("Stop motor");
  unsigned int len = 2;
  int result = write_port(end_port, len);
  laser_start_flag = false;
  usleep(10000);
  flush_port();
  return true;
}

bool start_motor(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &res) {
  boost::mutex::scoped_lock lock(mStausMutex_);
  if (laser_fd == -1)
    return true;
  ROS_DEBUG("Start motor");
  flush_port();
  unsigned int len = 2;
  int result = write_port(start_port, len);
  usleep(10000);
  laser_start_flag = true;
  return true;
}

int main(int argc, char **argv) {
  printf("sc_mini initiating\n");
  ros::init(argc, argv, "sc_mini");

  int baud_rate;
  int log_cnt = 0;
  lidar_start_count = 5;
  std::string frame_id;
  std::string port;

  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  sensor_msgs::LaserScan scan_publish;
  scan_publish.ranges.resize(360);
  scan_publish.intensities.resize(360);

  sensor_msgs::LaserScan scan;
  scan.ranges.resize(RANGES_TIMP_SIZE);
  // sensor_msgs::LaserScan::Ptr scan_publish(new sensor_msgs::LaserScan);

  ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 10);
  priv_nh.param("baud_rate", baud_rate, 115200);
  priv_nh.param("frame_id", frame_id, std::string("base_link"));
  priv_nh.param("port", port, std::string("/dev/sc_mini"));

  const char *port_t = port.data();

  // ros::ServiceServer stop_motor_service = n.advertiseService("stop_motor",
  // stop_motor);
  // ros::ServiceServer start_motor_service = n.advertiseService("start_motor",
  // start_motor);

  printf("openning port\n");
  laser_fd = open_port(port_t, 115200);
  if (laser_fd == -1) {
    return -1;
  }
  flush_port();
  sc_m_c::SCLaser laser;

  printf("sc_mini start\n");
  // 订阅里程计数据
  // ros::Subscriber sub = n.subscribe("odom", 1000, odomCallback);
  while (ros::ok()) {
    {
      boost::mutex::scoped_lock lock(mStausMutex_);
      if (laser_start_flag) {
        if (laser.poll(scan, laser_fd) == 0) {
          if (lidar_start_count > 0) // 丢弃前五圈激光雷达数据
          {
            lidar_start_count--;
          }
          if (lidar_start_count == 0) {
            laser.PointCloudFilter(
                scan); // note: 滤波函数，如果不用该滤波功能，将该行屏蔽即可。
            laser.angle_insert(scan, scan_publish); // note:角度插值函数
            scan_publish.header.frame_id = frame_id;
            scan_publish.header.stamp = ros::Time::now();
            laser_pub.publish(scan_publish);
          }
        }
      }
    }
    usleep(50000);
    // ros::spinOnce();
  }
  laser.stop();
  usleep(200000);
  close(laser_fd);
  return 0;
}

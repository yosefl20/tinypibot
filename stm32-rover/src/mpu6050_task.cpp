#include "mpu6050_task.h"

#include "msgs.h"
#include "uart_task.h"

// #define OUTPUT_REAL_READING
// #define OUTPUT_READABLE_QUATERNION
// #define OUTPUT_READABLE_EULER
// #define OUTPUT_READABLE_YAWPITCHROLL
// #define OUTPUT_READABLE_REALACCEL
// #define OUTPUT_READABLE_WORLDACCEL
// #define OUTPUT_TEAPOT

extern UartTask uartTask;

Mpu6050Task::Mpu6050Task()
    : m_dmpReady(false), m_accelScalar(1.0f), m_gyroScalar(1.0f) {}

Mpu6050Task::~Mpu6050Task() {}

void Mpu6050Task::setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  // fix stm32f103c8t6 i2c issue
  NVIC_SetPriority(TIM1_CC_IRQn, 0xFF);

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having
                         // compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.println(F("Initializing MPU6050..."));
  m_mpu.initialize();
  // pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(m_mpu.testConnection() ? F("MPU6050 connection successful")
                                        : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  m_devStatus = m_mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  m_mpu.setXGyroOffset(220);
  m_mpu.setYGyroOffset(76);
  m_mpu.setZGyroOffset(-85);
  m_mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (m_devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    m_mpu.CalibrateAccel(6);
    m_mpu.CalibrateGyro(6);
    m_mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    m_mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // Serial.print(
    //     F("Enabling interrupt detection (Arduino external interrupt "));
    // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    // Serial.println(F(")..."));
    // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady,
    // RISING);
    // m_mpuIntStatus = m_mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use
    // it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    m_dmpReady = true;

    // setup scalar
    m_accelScalar = 32767.0f / (float)(1 << m_mpu.getFullScaleAccelRange());
    m_gyroScalar = 32767.0f / (float)(1 << m_mpu.getFullScaleGyroRange());

    // get expected DMP packet size for later comparison
    m_packetSize = m_mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(m_devStatus);
    Serial.println(F(")"));
  }

  Serial.println("mpu6050 is ready");
}

unsigned long Mpu6050Task::loop(MicroTasks::WakeReason reason) {

  // if programming failed, don't try to do anything
  if (!m_dmpReady)
    return INFINITY;

  // read a packet from FIFO
  if (m_mpu.dmpGetCurrentFIFOPacket(m_fifoBuffer)) { // Get the Latest packet
    m_mpu.dmpGetQuaternion(&m_q, m_fifoBuffer);
    m_mpu.dmpGetGravity(&m_gravity, &m_q);
    m_mpu.dmpGetYawPitchRoll(m_ypr, &m_q, &m_gravity);
    m_mpu.dmpGetAccel(&m_aa, m_fifoBuffer);
    m_mpu.dmpGetGyro(&m_ag, m_fifoBuffer);

#ifdef OUTPUT_REAL_READING
    m_mpu.dmpGetEuler(m_euler, &m_q);
    m_mpu.dmpGetLinearAccel(&m_aaReal, &m_aa, &m_gravity);
    m_mpu.dmpGetLinearAccelInWorld(&m_aaWorld, &m_aaReal, &m_q);

    static const float gravity_value = 9.81;
    static const float deg_to_rad_factor = M_PI / 180.0;

    m_accel.x = m_aa.x * m_accelScalar * 2.0f * gravity_value;
    m_accel.y = m_aa.y * m_accelScalar * 2.0f * gravity_value;
    m_accel.z = m_aa.z * m_accelScalar * 2.0f * gravity_value;

    m_gyro.x = m_ag.x * m_gyroScalar * 250.0f * deg_to_rad_factor;
    m_gyro.y = m_ag.y * m_gyroScalar * 250.0f * deg_to_rad_factor;
    m_gyro.z = m_ag.z * m_gyroScalar * 250.0f * deg_to_rad_factor;
#endif

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    Serial.print("quat\t");
    Serial.print(m_q.w);
    Serial.print("\t");
    Serial.print(m_q.x);
    Serial.print("\t");
    Serial.print(m_q.y);
    Serial.print("\t");
    Serial.println(m_q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    Serial.print("euler\t");
    Serial.print(m_euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(m_euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(m_euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    Serial.print("ypr\t");
    Serial.print(m_ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(m_ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(m_ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    Serial.print("areal\t");
    Serial.print(m_aaReal.x);
    Serial.print("\t");
    Serial.print(m_aaReal.y);
    Serial.print("\t");
    Serial.println(m_aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    Serial.print("aworld\t");
    Serial.print(m_aaWorld.x);
    Serial.print("\t");
    Serial.print(m_aaWorld.y);
    Serial.print("\t");
    Serial.println(m_aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

    uartTask.send(new ImuMessage(
        m_aa.x, m_aa.y, m_aa.z, (1 << m_mpu.getFullScaleAccelRange()) * 2,
        m_ag.x, m_ag.y, m_ag.z, (1 << m_mpu.getFullScaleGyroRange()) * 250,
        (int16_t)(m_q.w * 16384.0f), (int16_t)(m_q.x * 16384.0f),
        (int16_t)(m_q.y * 16384.0f), (int16_t)(m_q.z * 16384.0f)));
  }

  return 5; // running at 200Hz
}
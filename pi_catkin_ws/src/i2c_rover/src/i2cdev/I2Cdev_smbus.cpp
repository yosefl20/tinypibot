#include "i2cdev/I2Cdev.h"
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>

static int i2csmbus_fp = NULL;

I2Cdev::I2Cdev() {}

void I2Cdev::initialize() {

  if (i2csmbus_fp == NULL) {
    i2csmbus_fp = open("/dev/i2c-1", O_RDWR);
  }
}

/** Enable or disable I2C,
 * @param isEnabled true = enable, false = disable
 */
void I2Cdev::enable(bool isEnabled) {}

char sendBuf[256];
char recvBuf[256];

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                       uint8_t *data) {

  // not implemented
  return 0;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any
 * bitStart position will equal 0x05)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
                        uint8_t length, uint8_t *data) {
  return 0;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data) {

  if (ioctl(i2csmbus_fp, I2C_SLAVE, devAddr) < 0) {
    return 0;
  }
  __s32 res;
  res = i2c_smbus_read_byte_data(i2csmbus_fp, regAddr);
  if (res == -1) {
    return 0;
  }
  data[0] = res & 0xff;
  return 1;
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @return I2C_TransferReturn_TypeDef
 * http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
 */
int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                         uint8_t *data) {

  if (ioctl(i2csmbus_fp, I2C_SLAVE, devAddr) < 0) {
    return 0;
  }
  __s32 res;
  res = i2c_smbus_read_i2c_block_data(i2csmbus_fp, regAddr, length, data);
  if (res == -1)
    return 0;
  return 1;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                      uint8_t data) {
  return false;
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
                       uint8_t length, uint8_t data) {
  return false;
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
  if (ioctl(i2csmbus_fp, I2C_SLAVE, devAddr) < 0) {
    return 0;
  }
  __s32 res;
  res = i2c_smbus_write_byte_data(i2csmbus_fp, regAddr, length, data);
  if (res == -1)
    return 0;
  return 1;
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data) {
  if (ioctl(i2csmbus_fp, I2C_SLAVE, devAddr) < 0) {
    return 0;
  }
  __s32 res;
  res = i2c_smbus_read_word_data(i2csmbus_fp, regAddr);
  if (res == -1)
    return 0;
  data[0] = res & 0xffff;
  return 1;
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @return Number of words read (-1 indicates failure)
 */
int8_t I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                         uint16_t *data) {
  if (ioctl(i2csmbus_fp, I2C_SLAVE, devAddr) < 0) {
    return 0;
  }
  __s32 res;
  res = i2c_smbus_read_i2c_block_data(i2csmbus_fp, regAddr, length * 2,
                                      (uint8_t *)data);
  if (res == -1)
    return 0;
  return 1;
}

bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
  if (ioctl(i2csmbus_fp, I2C_SLAVE, devAddr) < 0) {
    return false;
  }
  __s32 res;
  res = i2c_smbus_write_block_data(i2csmbus_fp, regAddr, 2, (uint8_t *)&data);
  if (res == -1)
    return false;
  return true;
}

bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                        uint8_t *data) {
  if (ioctl(i2csmbus_fp, I2C_SLAVE, devAddr) < 0) {
    return false;
  }
  __s32 res;
  res = i2c_smbus_write_block_data(i2csmbus_fp, regAddr, length, data);
  if (res == -1)
    return false;
  return true;
}

bool I2Cdev::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                        uint16_t *data) {
  if (ioctl(i2csmbus_fp, I2C_SLAVE, devAddr) < 0) {
    return false;
  }
  __s32 res;
  res = i2c_smbus_write_block_data(i2csmbus_fp, regAddr, length * 2,
                                   (uint8_t *)data);
  if (res == -1)
    return false;
  return true;
}
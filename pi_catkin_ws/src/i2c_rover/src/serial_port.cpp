#include "serial_port.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

SerialPort::SerialPort() : m_fd(NULL) {}

SerialPort::~SerialPort() { close(); }

int SerialPort::setOpt(int nSpeed, int nBits, char nEvent, int nStop) {

  struct termios newtio, oldtio;

  if (tcgetattr(m_fd, &oldtio) != 0) {
    perror("SetupSerial 1");

    return -1;
  }
  bzero(&newtio, sizeof(newtio));

  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  switch (nBits) {
  case 7:
    newtio.c_cflag |= CS7;
    break;
  case 8:
    newtio.c_cflag |= CS8;
    break;
  }

  switch (nEvent) {
  case 'o':
  case 'O':
    newtio.c_cflag |= PARENB;
    newtio.c_cflag |= PARODD;
    newtio.c_iflag |= (INPCK | ISTRIP);
    break;
  case 'e':
  case 'E':
    newtio.c_iflag |= (INPCK | ISTRIP);
    newtio.c_cflag |= PARENB;
    newtio.c_cflag &= ~PARODD;
    break;
  case 'n':
  case 'N':
    newtio.c_cflag &= ~PARENB;
    break;
  default:
    break;
  }

  switch (nSpeed) {
  case 2400:
    cfsetispeed(&newtio, B2400);
    cfsetospeed(&newtio, B2400);
    break;
  case 4800:
    cfsetispeed(&newtio, B4800);
    cfsetospeed(&newtio, B4800);
    break;
  case 9600:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  case 115200:
    cfsetispeed(&newtio, B115200);
    cfsetospeed(&newtio, B115200);
    break;
  case 460800:
    cfsetispeed(&newtio, B460800);
    cfsetospeed(&newtio, B460800);
    break;
  default:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  }

  if (nStop == 1)
    newtio.c_cflag &= ~CSTOPB;
  else if (nStop == 2)
    newtio.c_cflag |= CSTOPB;

  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;

  tcflush(m_fd, TCIFLUSH);

  if ((tcsetattr(m_fd, TCSANOW, &newtio)) != 0) {
    perror("com set error");
    return -1;
  }

  return 0;
}

int SerialPort::open(const char *port, int baud_rate) {
  if (access(port, F_OK) != 0) //  /dev/ttyS1
  {
    printf("Can't access port: %s, exit node lidar.\r\n", port);
    return -1;
  }
  m_fd = ::open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (m_fd > 0) {
    if (0 == setOpt(baud_rate, 8, 'N', 1)) {
      tcflush(m_fd, TCIOFLUSH);
      // LOGI("open_port ok.\r\n");
    } else {
      // LOGI("set baudrate fail, open_port fail.\r\n");
      return -1;
    }
  } else {
    perror("Can't Open SerialPort");
  }
  return m_fd;
}

int SerialPort::read(unsigned char *pbuf, unsigned int len) {
  int rlen = 0;
  if (m_fd > 0) {
    rlen = ::read(m_fd, pbuf, len);
  } else {
    perror("Can't Open SerialPort");
  }
  return rlen;
}

int SerialPort::write(unsigned char *pbuf, unsigned int len) {
  int wlen = 0;
  if (m_fd > 0) {
    wlen = ::write(m_fd, pbuf, len);
  } else {
    perror("Can't Open SerialPort");
  }
  return wlen;
}

int SerialPort::writeAll(unsigned char *pbuf, unsigned int len) {
  int written = 0;
  while (written < len) {
    int tlen = write(pbuf + written, len - written);
    if (tlen < 0)
      break;
    written += tlen;
  }
  return written;
}

int SerialPort::close() {
  if (m_fd > 0) {
    ::close(m_fd);
    m_fd = NULL;
    return 0;
  }
  return -1;
}

int SerialPort::flush() {
  if (m_fd > 0) {
    tcflush(m_fd, TCIOFLUSH);
  } else {
    return -1;
  }
  return 0;
}

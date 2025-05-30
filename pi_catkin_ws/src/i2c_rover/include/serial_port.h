#pragma once
#ifndef _SERIAL_PORT_H
#define _SERIAL_PORT_H

class SerialPort {
public:
  SerialPort();
  ~SerialPort();

  int open(const char *port, int baud_rate);
  int read(unsigned char *pbuf, unsigned int len);
  int write(unsigned char *pbuf, unsigned int len);
  int writeAll(unsigned char *pbuf, unsigned int len);
  int close();
  int flush();

  bool isOpen() const { return m_fd; }

protected:
  int setOpt(int nSpeed, int nBits, char nEvent, int nStop);

private:
  int m_fd;
};

#endif //_SERIAL_PORT_H
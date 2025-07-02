#ifndef _SERIAL_PORT_H
#define _SERIAL_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

int open_port(const char *port, int baud_rate);
int read_port(unsigned char *pbuf, unsigned int len);
int write_port(unsigned char *pbuf, unsigned int len);
int close_port();
int flush_port();

#ifdef __cplusplus
}
#endif

#endif
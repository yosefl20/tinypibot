#pragma once

#define PACKET_HEADER 0xE7
#define UART_PACKET_LEN 12

#pragma pack(push, 1)

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

#pragma pack(pop)
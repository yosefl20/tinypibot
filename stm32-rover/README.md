# stm32 rover

## IIC communicate protocols

### slave address: 0x10

### pi -> stm32

register | value
-------- | --------------
0x01     | left motor throttle(int8)
0x02     | right motor throttle(int8)
0x03     | linear speed, cm/s(int8)
0x04     | angular speed, degree/s(int8)
0x8x     | register addr to read


### stm32 -> pi

register | value
-------- | --------------
0x81     | left encoder pos(int32)
0x82     | right encoder pos(int32)
0x83     | ultrasonic reading(uint8)
0x84     | battery percentage(uint8)
0x85     | LR speed(int16+int16)

## UART communicate protocols

### common struct

offset | len  | type | value
------ | ---- | ---- | --------
00H    | 1    | Byte | header(0xE7)
01H    | 1    | Byte | cmd
02H    | 8    |      | payloads
0AH    | 1    | Byte | checksum

### pi -> stm32

       name      | cmd      | payload
---------------- | -------- | --------------
       throttle  | 0x01     | left motor(int32), right motor(int32). value: -255 ~ 255
       cmd_vel   | 0x02     | left speed(ticks/sec, float32), right speed(ticks/sec, float32)
query ultrasonic | 0x03     | /         
  query battery  | 0x04     | /
  query speed    | 0x05     | /

### stm32 -> pi

  name     | cmd      | payload
---------- | -------- | --------------
encoder pos| 0x82     | left pos(ticks, int32), right pos(ticks, int32)
ultrasonic | 0x83     | ul0 distance(cm, int16), ul1 distance(cm, int16), ul2 distance(cm, int16), ul3 distance(cm, int16)
  battery  | 0x84     | voltage(v, float32), percentage(%, float32)
    speeds | 0x85     | left encoder speed(ticks/sec, float32), right encoder speed(ticks/sec, float32)
imu_quaterion | 0x86     | w,x,y,z : int16, /16384.0
accel_raw  | 0x87     | x,y,z,r : int16, r = range, ±g
gyro_raw   | 0x88     | x,y,z,r : int16, r = range, ±deg/s
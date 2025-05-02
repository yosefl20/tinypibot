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

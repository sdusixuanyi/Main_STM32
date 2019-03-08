#ifndef _I2C_H_
#define _I2C_H_

#include "stm32f10x.h"
#include "math.h"

#define I2C1_OWN_ADDRESS 0x0A
#define I2C2_OWN_ADDRESS 0x0A
#define I2C_Speed        200000               //初始化中设置的传输速率

void i2c_init(void);

void i2c_byte_write(unsigned char slave_addr, unsigned char reg_char, unsigned char data);

void i2c_read(unsigned char slave_addr, unsigned char reg_char, unsigned char* data, int count);

#endif

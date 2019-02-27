#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "stm32f10x.h"
#include "math.h"
#include "i2c.h"
#include "myMath.h"
#include "flight_control.h"

#define MPU6050_ADDRESS  0xD0             //11010000B   高7位数字固定，最后一位与芯片的AD0接的高低电平有关，此处接地

//MPU6050内部寄存器地址
#define MPU6050_RA_PWR_MGMT_1   0x6B          //电源模式和时钟源
#define MPU6050_RA_SMPLRT_DIV   0x19          //采样频率分频     采样率 = 陀螺仪输出率 / (1 + SMPLRT_DIV)
#define MPU6050_RA_CONFIG       0x1A          //加速度计和陀螺仪外帧同步和数字滤波
#define MPU6050_RA_ACCEL_CONFIG 0x1C          //加速度计自检和量程设置
#define MPU6050_RA_GYRO_CONFIG  0x1B          //陀螺仪自检和量程设置

//MPU6050数据存放地址
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H        0x41
#define MPU6050_RA_TEMP_OUT_L        0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

//量程设置
#define ACCEL_RANGE  4       //单位g
#define GYRO_RANGE   2000    //单位°/s


void MPU6050_init(void);

void MPU6050_get_data(unsigned char* addr);

void MPU6050_data_translation(unsigned char* raw_addr, short int* translation_addr);

void MPU6050_dma_read(unsigned char slave_addr, unsigned char reg_addr);

int MPU6050_check(void);         //MPU6050自检程序，返回1表示数据读取正常，返回0表示异常，需要I2C初始化后再使用

#endif

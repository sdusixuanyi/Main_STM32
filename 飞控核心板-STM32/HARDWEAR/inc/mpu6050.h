#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "stm32f10x.h"
#include "math.h"
#include "i2c.h"
#include "myMath.h"
#include "flight_control.h"

#define MPU6050_ADDRESS  0xD0             //11010000B   ��7λ���̶ֹ������һλ��оƬ��AD0�ӵĸߵ͵�ƽ�йأ��˴��ӵ�

//MPU6050�ڲ��Ĵ�����ַ
#define MPU6050_RA_PWR_MGMT_1   0x6B          //��Դģʽ��ʱ��Դ
#define MPU6050_RA_SMPLRT_DIV   0x19          //����Ƶ�ʷ�Ƶ     ������ = ����������� / (1 + SMPLRT_DIV)
#define MPU6050_RA_CONFIG       0x1A          //���ٶȼƺ���������֡ͬ���������˲�
#define MPU6050_RA_ACCEL_CONFIG 0x1C          //���ٶȼ��Լ����������
#define MPU6050_RA_GYRO_CONFIG  0x1B          //�������Լ����������

//MPU6050���ݴ�ŵ�ַ
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

//��������
#define ACCEL_RANGE  4       //��λg
#define GYRO_RANGE   2000    //��λ��/s


void MPU6050_init(void);

void MPU6050_get_data(unsigned char* addr);

void MPU6050_data_translation(unsigned char* raw_addr, short int* translation_addr);

void MPU6050_dma_read(unsigned char slave_addr, unsigned char reg_addr);

int MPU6050_check(void);         //MPU6050�Լ���򣬷���1��ʾ���ݶ�ȡ����������0��ʾ�쳣����ҪI2C��ʼ������ʹ��

#endif

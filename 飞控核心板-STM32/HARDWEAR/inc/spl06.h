#ifndef _SPL06_H_
#define _SPL06_H_

#include "i2c.h"
#include "math.h"
#include "flight_control.h"
#include "delay.h"

#define SPL06_ADDR 0x76 << 1   //��ѹ�ƴӻ���ַ��ʹ��ʱ��Ҫ����һλȻ�����һλ��1��0
//#define ID_REG 0x0D        //��ѹ��ID�Ĵ�����ַ
#define PRODUCT_ID 0X10    //��ƷID

//�Ĵ�������
#define PRESSURE_REG 0X00
#define TEMP_REG 0X03
#define PRS_CFG_REG 0x06 //��ѹ������������
#define TMP_CFG_REG 0x07 //�¶Ȳ����ٶ�����
#define MEAS_CFG_REG 0x08 //���������봫��������
#define CFG_REG 0x09 //�ж�/FIFO/SPI����������
#define INT_STS_REG 0X0A //�ж�״̬��־λ
#define FIFO_STS_REG 0X0B //FIFO״̬
#define RESET_REG 0X0C
#define ID_REG 0x0D
#define COEF_REG 0x10

#define MEAS_CFG_MEAS_CTR_STANDBY 0 //ģʽ���� ����ģʽ
#define MEAS_CFG_MEAS_CTR_COMMAND_PRS 0x01 //ģʽ���� ����ģʽ��������ѹ�ɼ�
#define MEAS_CFG_MEAS_CTR_COMMAND_TMP 0x02 //ģʽ���� ����ģʽ�������¶Ȳɼ�
#define MEAS_CFG_MEAS_CTR_BACKGROUND_PRS 0x05 //ģʽ���� ��̨ģʽֻ��ȡ��ѹֵ
#define MEAS_CFG_MEAS_CTR_BACKGROUND_TMP 0X06 //ģʽ���� ��̨ģʽֻ��ȡ�¶�ֵ
#define MEAS_CFG_MEAS_CTR_BACKGROUND_PSR_TMP 0X07 //ģʽ���� ��̨ģʽͬʱ��ȡ�¶�ֵ����ѹֵ

#define MEAS_CFG_COEF_RDY 0X80 // �������ڲ�У׼ֵ�ɶ�����������
#define MEAS_CFG_SENSOR_RDY 0X40 // �������ѳ�ʼ����ɣ���������
#define MEAS_CFG_TMP_RDY 0x20 //�¶�ֵ�Ѿ�׼�����������Խ��ж�ȡ���ñ�־λ��ȡ���Զ���0
#define MEAS_CFG_PRS_RDY 0x10 //��ѹֵ�Ѿ�׼�����������Խ��ж�ȡ���ñ�־λ


#define PRESSURE_SENSOR 0
#define TEMPERATURE_SENSOR 1

#define SPL06_HARDWARE 0
#define SPL06_SOFTWARE 1

struct calib_param
{
	short c0;
	short c1;
	int c00;
	int c10;
	short c01;
	short c11;
	short c20;
	short c21;
	short c30;
};

struct SPL06_DATA
{
	int i32RawPressure;
	int i32RawTemperature;
	int i32KP;
	int i32KT;
	
	float fGround_Alt;       //����ĺ���
	float fALT;              //���β�õĺ���
	float fRelative_Alt;     //��Ժ��Σ����ɻ��������ĸ߶�
	
	float fTemperature;
	float fPressure;
	float fLast_Pressure;
	
	float fOffset;
};

int SPL06_check(int i2cmode);

void SPL06_init(int i2cmode);

void SPL06_get_raw_data(int i2cmode);

void SPL06_set_rate(unsigned char u8_Sensor, unsigned char u8_SmplRate, unsigned char u8_OverSmpl, int i2cmode);

void SPL06_get_calib_param(int mode, int i2cmode);    //��ȡУ׼ֵ��0Ϊ�˹����ã�1Ϊ�Զ���ȡ

void SPL06_height_process(int i2cmode);      //��ȡ�߶Ȳ�����attitude.height��

float SPL06_get_pressure(void);

void SPL06_info_update(int i2cmode);

#endif

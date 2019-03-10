#ifndef _SPL06_H_
#define _SPL06_H_

#include "i2c.h"
#include "math.h"
#include "flight_control.h"
#include "delay.h"

#define SPL06_ADDR 0x76 << 1   //气压计从机地址，使用时需要左移一位然后将最后一位置1或0
//#define ID_REG 0x0D        //气压计ID寄存器地址
#define PRODUCT_ID 0X10    //产品ID

//寄存器定义
#define PRESSURE_REG 0X00
#define TEMP_REG 0X03
#define PRS_CFG_REG 0x06 //气压测量速率配置
#define TMP_CFG_REG 0x07 //温度测量速度配置
#define MEAS_CFG_REG 0x08 //测量配置与传感器配置
#define CFG_REG 0x09 //中断/FIFO/SPI线数等配置
#define INT_STS_REG 0X0A //中断状态标志位
#define FIFO_STS_REG 0X0B //FIFO状态
#define RESET_REG 0X0C
#define ID_REG 0x0D
#define COEF_REG 0x10

#define MEAS_CFG_MEAS_CTR_STANDBY 0 //模式配置 挂起模式
#define MEAS_CFG_MEAS_CTR_COMMAND_PRS 0x01 //模式配置 命令模式下启动气压采集
#define MEAS_CFG_MEAS_CTR_COMMAND_TMP 0x02 //模式配置 命令模式下启动温度采集
#define MEAS_CFG_MEAS_CTR_BACKGROUND_PRS 0x05 //模式配置 后台模式只读取气压值
#define MEAS_CFG_MEAS_CTR_BACKGROUND_TMP 0X06 //模式配置 后台模式只读取温度值
#define MEAS_CFG_MEAS_CTR_BACKGROUND_PSR_TMP 0X07 //模式配置 后台模式同时读取温度值和气压值

#define MEAS_CFG_COEF_RDY 0X80 // 传感器内部校准值可读，在启动后
#define MEAS_CFG_SENSOR_RDY 0X40 // 传感器已初始化完成，在启动后
#define MEAS_CFG_TMP_RDY 0x20 //温度值已经准备就绪，可以进行读取，该标志位读取后自动清0
#define MEAS_CFG_PRS_RDY 0x10 //气压值已经准备就绪，可以进行读取，该标志位


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
	
	float fGround_Alt;       //地面的海拔
	float fALT;              //本次测得的海拔
	float fRelative_Alt;     //相对海拔，即飞机距离地面的高度
	
	float fTemperature;
	float fPressure;
	float fLast_Pressure;
	
	float fOffset;
};

int SPL06_check(int i2cmode);

void SPL06_init(int i2cmode);

void SPL06_get_raw_data(int i2cmode);

void SPL06_set_rate(unsigned char u8_Sensor, unsigned char u8_SmplRate, unsigned char u8_OverSmpl, int i2cmode);

void SPL06_get_calib_param(int mode, int i2cmode);    //获取校准值，0为人工配置，1为自动读取

void SPL06_height_process(int i2cmode);      //获取高度并存入attitude.height中

float SPL06_get_pressure(void);

void SPL06_info_update(int i2cmode);

#endif

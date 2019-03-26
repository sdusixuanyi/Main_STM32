#ifndef _I2C_H_
#define _I2C_H_

//#include "stm32f10x.h"
//#include "math.h"
//#include "mpu6050.h"
//#include "spl06.h"

#include "headfile.h"

#define I2C1_OWN_ADDRESS 0x0A
#define I2C2_OWN_ADDRESS 0x0A
#define I2C_Speed        200000               //��ʼ�������õĴ�������

#define I2C_HARDWARE 0
#define I2C_SOFTWARE 1

//IO��������
#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

//IO��������	 
#define IIC_SCL    PBout(10) //SCL
#define IIC_SDA    PBout(11) //SDA	 
#define READ_SDA   PBin(11)  //����SDA 

void i2c_init(int mode);

void i2c_byte_write(unsigned char slave_addr, unsigned char reg_char, unsigned char data);

void i2c_read(unsigned char slave_addr, unsigned char reg_char, unsigned char* data, int count);


//ģ��IIC����
void IIC_Init(void);                //��ʼ��IIC��IO��				 

void IIC_Start(void);				//����IIC��ʼ�ź�

void IIC_Stop(void);	  			//����IICֹͣ�ź�

void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�

u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�

u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�

void IIC_Ack(void);					//IIC����ACK�ź�

void IIC_NAck(void);				//IIC������ACK�ź�

u8 slave_addr_process(u8 slave_addr);

void i2c_byte_write_soft(unsigned char slave_addr, unsigned char reg_addr, unsigned char data);

void i2c_read_soft(unsigned char slave_addr, unsigned char reg_addr, unsigned char* data, int count);

//**********************************************��ֲ������ԭ��IIC����*********************************************************
//IO��������	 
#define MPU_IIC_SCL    PBout(10) 		//SCL
#define MPU_IIC_SDA    PBout(11) 		//SDA	 
#define MPU_READ_SDA   PBin(11) 		//����SDA 

#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

//IIC���в�������
void MPU_IIC_Delay(void);				//MPU IIC��ʱ����
//void MPU_IIC_Init(void);                //��ʼ��IIC��IO��				 
void MPU_IIC_Start(void);				//����IIC��ʼ�ź�
void MPU_IIC_Stop(void);	  			//����IICֹͣ�ź�
void MPU_IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 MPU_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 MPU_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void MPU_IIC_Ack(void);					//IIC����ACK�ź�
void MPU_IIC_NAck(void);				//IIC������ACK�ź�

void IMPU_IC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	 
//***************************************************************************************************************************
#endif

#ifndef __KEY_H
#define __KEY_H	 
//#include "sys.h"
#include "headfile.h"	 
 
#define KEY0  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)//��ȡ����0
#define KEY1  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14)//��ȡ����1
 
#define KEY0_PRES	1		//KEY0  
#define KEY1_PRES	2		//KEY1 
#define KEY2_PRES	3		//KEY1 

void KEY_Init(void);	//IO��ʼ��
u8 KEY_Scan(u8 mode);  	//����ɨ�躯��������ĳ�������Ƿ񱻰���					    
#endif

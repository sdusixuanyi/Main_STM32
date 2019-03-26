#ifndef __KEY_H
#define __KEY_H	 
//#include "sys.h"
#include "headfile.h"	 
 
#define KEY0  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)//读取按键0
#define KEY1  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14)//读取按键1
 
#define KEY0_PRES	1		//KEY0  
#define KEY1_PRES	2		//KEY1 
#define KEY2_PRES	3		//KEY1 

void KEY_Init(void);	//IO初始化
u8 KEY_Scan(u8 mode);  	//按键扫描函数，返回某个按键是否被按下					    
#endif

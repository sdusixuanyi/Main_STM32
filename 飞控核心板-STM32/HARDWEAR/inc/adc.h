#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"


void Adc_Init(void);//初始化函数，将ADC—CH1通道初始化
u16  Get_Adc(u8 ch); //得到一次ADC采集的值，ch为通道号1
u16 Get_Adc_Average(u8 ch,u8 times); //求取五次采样的平均值，ch为通道号1
 
#endif 

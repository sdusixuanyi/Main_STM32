#ifndef _BEEP_H_
#define _BEEP_H_
//#include "sys.h"
#include "headfile.h"


void Beep_Init(void);//蜂鸣器初始化函数
void Beep_ON(u8 mode,u8 num);//打开蜂鸣器并按照mode（1,2,3）和num1~7进行选择
void Beep_OFF(void );//关闭蜂鸣器


#endif


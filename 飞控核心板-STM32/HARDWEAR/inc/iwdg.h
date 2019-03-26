#ifndef __WDG_H
#define __WDG_H
//#include "sys.h"

#include "headfile.h"
 
void IWDG_Init(u8 prer,u16 rlr);//4,625看门狗初始化程序，注意一旦初始化就无法返回关闭，复位后也会一直运行，除非上电否则必须一直喂狗
void IWDG_Feed(void);//喂狗函数，上述参数选择条件下必须一秒内调用一次函数

 
#endif

#ifndef _TIMER_H_
#define _TIMER_H_

//#include "stm32f10x.h"
//#include "usart.h"

#include "headfile.h"

#define Period_timer2 360          //非dma模式280大约为极限  频率为36000 / 280    dma模式极限20(从申请dma到传输完成不要有printf等耗时长的操作)

struct Sys_flag
{
	u8 SendDataPermission;            //控制飞控姿态数据发送的标志位
	u8 AttitudeProcessPermission;        //控制是否进行数据解算的标志位
	u32 AttitudeProcessOvertimeErr;      //超时计数
};

void timer3_nvic_init(void);

void TIM3_init(void);

void timer3_init(void);

void SYSFLAG_init(void);

void SYSFLAG_update(void);

#endif


#ifndef _TIMER_H_
#define _TIMER_H_

#include "stm32f10x.h"
#include "usart.h"

#define Period_timer2 360          //非dma模式280大约为极限  频率为36000 / 280    dma模式极限20(从申请dma到传输完成不要有printf等耗时长的操作)

struct Status_flag
{
	u8 data_send;
	u8 attitude_process;
};

void timer3_nvic_init(void);

void TIM3_init(void);

void timer3_init(void);

#endif


#ifndef _TIMER_H_
#define _TIMER_H_

#include "stm32f10x.h"
#include "usart.h"

#define Period_timer2 360          //��dmaģʽ280��ԼΪ����  Ƶ��Ϊ36000 / 280    dmaģʽ����20(������dma��������ɲ�Ҫ��printf�Ⱥ�ʱ���Ĳ���)

struct Status_flag
{
	u8 data_send;
	u8 attitude_process;
};

void timer3_nvic_init(void);

void TIM3_init(void);

void timer3_init(void);

#endif


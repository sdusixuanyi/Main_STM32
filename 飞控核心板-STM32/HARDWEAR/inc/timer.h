#ifndef _TIMER_H_
#define _TIMER_H_

//#include "stm32f10x.h"
//#include "usart.h"

#include "headfile.h"

#define Period_timer2 360          //��dmaģʽ280��ԼΪ����  Ƶ��Ϊ36000 / 280    dmaģʽ����20(������dma��������ɲ�Ҫ��printf�Ⱥ�ʱ���Ĳ���)

struct Sys_flag
{
	u8 SendDataPermission;            //���Ʒɿ���̬���ݷ��͵ı�־λ
	u8 AttitudeProcessPermission;        //�����Ƿ�������ݽ���ı�־λ
	u32 AttitudeProcessOvertimeErr;      //��ʱ����
};

void timer3_nvic_init(void);

void TIM3_init(void);

void timer3_init(void);

void SYSFLAG_init(void);

void SYSFLAG_update(void);

#endif


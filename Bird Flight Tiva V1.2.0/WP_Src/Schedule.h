#ifndef __SCHEDULE_H__
#define __SCHEDULE_H__

#include "WP_DataType.h"
extern uint32 TIME_ISR_CNT;
extern void TIMER1A_Handler(void);
void Schedule_init(void);
void Test_Period(Testime *Time_Lab);

#endif


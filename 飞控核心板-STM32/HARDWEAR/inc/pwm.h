#ifndef __PWM_H
#define __PWM_H
#include "sys.h"

void TIM4_PWM_Init(u16 arr,u16 psc);//初始化函数，输入分频因子和分频数
void TIM4_CH1_Duty(u16 duty);//设置通道1占空比
void TIM4_CH2_Duty(u16 duty);//设置通道2占空比
void TIM4_CH3_Duty(u16 duty);//设置通道3占空比
void TIM4_CH4_Duty(u16 duty);//设置通道4占空比

#endif

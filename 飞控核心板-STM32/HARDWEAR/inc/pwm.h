#ifndef __PWM_H
#define __PWM_H
#include "sys.h"

void TIM4_PWM_Init(u16 arr,u16 psc);//��ʼ�������������Ƶ���Ӻͷ�Ƶ��
void TIM4_CH1_Duty(u16 duty);//����ͨ��1ռ�ձ�
void TIM4_CH2_Duty(u16 duty);//����ͨ��2ռ�ձ�
void TIM4_CH3_Duty(u16 duty);//����ͨ��3ռ�ձ�
void TIM4_CH4_Duty(u16 duty);//����ͨ��4ռ�ձ�

#endif

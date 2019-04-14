#include "timer.h"
#include "hcsr04.h"
#include "pwm.h"
#include "delay.h"
long int cycle = 0;            //���������������
struct Status_flag status = {0,0};
extern float motor_duty[4];
extern unsigned char raw_data[14];
extern short int translated_data[7];
extern int usart1_permission;
extern int tx_mode;
extern float accel[3];          //��ǰ����ֱ�Ϊx,y,z�ļ��ٶȷ���
extern float gyro[3];
extern float kalman_angle[3];
extern float accel_angle[3];

int TIM3_flag = 0;

void timer3_nvic_init(void)
{
	NVIC_InitTypeDef NVIC_InitStrue;
	
	NVIC_InitStrue.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitStrue);
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

void TIM3_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);           //72MHz
	
	TIM_TimeBaseStructure.TIM_Period = Period_timer2;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
  TIM_TimeBaseStructure.TIM_Prescaler = 1999;	    //����Ԥ��Ƶ������Ԥ��Ƶ����Ϊ72MHz����ʱ��Ƶ�� = ʱ��Ƶ�� / ��Ԥ��Ƶ + 1��
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);    //���ö�ʱ��
	
	TIM_Cmd(TIM3, DISABLE);                   //ʹ�ܶ�ʱ��2
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{	
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		
		TIM3_flag = 1;	

    status.data_send = 1;
    status.attitude_process = 1;		

		cycle++;
	}
}

void timer3_init(void)
{
	TIM3_init();	
	timer3_nvic_init();
	TIM_Cmd(TIM3, ENABLE);	
}



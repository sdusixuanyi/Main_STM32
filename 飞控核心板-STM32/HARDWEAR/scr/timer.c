#include "timer.h"
#include "hcsr04.h"
#include "pwm.h"
#include "delay.h"
long int cycle = 0;            //程序的运行周期数
struct Status_flag status = {0,0};
extern float motor_duty[4];
extern unsigned char raw_data[14];
extern short int translated_data[7];
extern int usart1_permission;
extern int tx_mode;
extern float accel[3];          //从前到后分别为x,y,z的加速度分量
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
	
	TIM_TimeBaseStructure.TIM_Period = Period_timer2;       //当定时器从0计数到999，即为1000次，为一个定时周期
  TIM_TimeBaseStructure.TIM_Prescaler = 1999;	    //设置预分频：若不预分频，即为72MHz，计时器频率 = 时钟频率 / （预分频 + 1）
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);    //配置定时器
	
	TIM_Cmd(TIM3, DISABLE);                   //使能定时器2
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



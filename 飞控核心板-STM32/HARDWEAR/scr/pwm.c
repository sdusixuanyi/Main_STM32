#include "pwm.h"
#include "led.h"
 
//PWM初始化
//arr:分频因子
//psc:分频数

 void TIM4_PWM_Init(u16 arr,u16 psc)
{                                                          
        GPIO_InitTypeDef GPIO_InitStructure;
        TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
        TIM_OCInitTypeDef  TIM_OCInitStructure;

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 |GPIO_Pin_8 |GPIO_Pin_9; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        GPIO_PinRemapConfig(GPIO_Remap_TIM4,DISABLE);

        TIM_TimeBaseStructure.TIM_Period = arr; 
        TIM_TimeBaseStructure.TIM_Prescaler =psc; 
        TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1; 
        TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
        
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;       
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
        TIM_OCInitStructure.TIM_Pulse = 0; 
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
        TIM_OC1Init(TIM4, &TIM_OCInitStructure);  
				TIM_OC2Init(TIM4, &TIM_OCInitStructure);
				TIM_OC3Init(TIM4, &TIM_OCInitStructure);
				TIM_OC4Init(TIM4, &TIM_OCInitStructure);

        TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  
				TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); 
				TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 
				TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); 
        TIM_ARRPreloadConfig(TIM4, ENABLE); 

		TIM_Cmd(TIM4, ENABLE);                                                                                     
}


void TIM4_CH1_Duty(u16 duty)//通道1输出对应占空比PWM波
{
	if(duty>=950)
		duty=950;
	TIM_SetCompare1(TIM4,duty*9);
}

void TIM4_CH2_Duty(u16 duty)//通道2输出对应占空比PWM波
{
	if(duty>=950)
		duty=950;
	TIM_SetCompare2(TIM4,duty*9);
}
void TIM4_CH3_Duty(u16 duty)//通道3输出对应占空比PWM波
{
	if(duty>=950)
		duty=950;
	TIM_SetCompare3(TIM4,duty*9);
}
void TIM4_CH4_Duty(u16 duty)//通道4输出对应占空比PWM波
{
	if(duty>=950)
		duty=950;
	TIM_SetCompare4(TIM4,duty*9);
}

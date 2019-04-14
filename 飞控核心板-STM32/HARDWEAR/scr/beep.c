#include "beep.h"
#include "delay.h"
#include "time.h"

const u16 tone[]={200,
	262,294,330,349,392,440,523,//????
	530,587,659,698,784,880,988,//????
	1047,1175,1319,1397,1568,1760,1967//????
};

void TIM3_PWM_Init(u16 arr,u16 psc);
void Beep_Init(void)
{
	TIM3_PWM_Init(8999,71);//72mhz/(8999+1)*(71+1)
}

//??????3????TIM3-CH4 PB1
void TIM3_PWM_Init(u16 arr,u16 psc)
{                                                          
        GPIO_InitTypeDef GPIO_InitStructure;
        TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
        TIM_OCInitTypeDef  TIM_OCInitStructure;

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);

        TIM_TimeBaseStructure.TIM_Period = arr; 
        TIM_TimeBaseStructure.TIM_Prescaler =psc; 
        TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
        
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;       
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
        TIM_OCInitStructure.TIM_Pulse = 0; 
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OC4Init(TIM3, &TIM_OCInitStructure);

		TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable); 
        TIM_ARRPreloadConfig(TIM3, ENABLE); 

		TIM_Cmd(TIM3, ENABLE);                                                                                     
}


//?????,mode??,?,??????,num??????
void Beep_ON(u8 mode,u8 num)
{
	u16 fhz=0,tone_num;
	tone_num=mode+num*7;
	TIM_Cmd(TIM3, ENABLE);
	switch(tone_num)
	{
		case 1:fhz=tone[1];break;
		case 2:fhz=tone[2];break;
		case 3:fhz=tone[3];break;
		case 4:fhz=tone[4];break;
		case 5:fhz=tone[5];break;
		case 6:fhz=tone[6];break;
		case 7:fhz=tone[7];break;
		case 8:fhz=tone[8];break;
		case 9:fhz=tone[9];break;
		case 10:fhz=tone[10];break;
		case 11:fhz=tone[11];break;
		case 12:fhz=tone[12];break;
		case 13:fhz=tone[13];break;
		case 14:fhz=tone[14];break;
		case 15:fhz=tone[15];break;
		case 16:fhz=tone[16];break;
		case 17:fhz=tone[17];break;
		case 18:fhz=tone[18];break;
		case 19:fhz=tone[19];break;
		case 20:fhz=tone[20];break;
		case 21:fhz=tone[21];break;
		default :fhz=tone[0];
	}
	TIM_SetAutoreload(TIM3,(1000000/fhz)-1);
	TIM_SetCompare4(TIM3,((1000000/fhz)-1)/2+1);
}

//?????,?????????????
void Beep_OFF(void )
{
	TIM_SetCompare4(TIM3,0);
	TIM_Cmd(TIM3, DISABLE);
}


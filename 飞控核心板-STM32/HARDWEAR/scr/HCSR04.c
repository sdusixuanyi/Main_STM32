#include "hcsr04.h"
#include "delay.h"
#include "time.h"

#define TRIG	GPIO_Pin_15//控制信号，PB15
#define ECHO	GPIO_Pin_14//返回信号，PB14

#define AVG_NUM 5
#define Distance_Offset 6.1f

float HCSR04_Distance=0.0;

void HCSR04_Init(void)
{
	
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	TIM2_Int_Init(30000,71);		//做输入捕捉模块，进行超声波信号捕捉

//配置gpio模块进行输出控制信号	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = TRIG;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	
//配置中断模块，进行接收信号捕捉中断	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能b口时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//使能复用功能
  /* Configure PB.14 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = ECHO;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);
  /* Configure EXTI14 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
 
  /* Enable and set EXTI9_5 Interrupt to the lowest priority */
//#include "RTE_Components.h"             // Component selection
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//?????
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
}
void HCSR04_Run(void)
{
		GPIO_WriteBit(GPIOB, GPIO_Pin_15, (BitAction)1);
		delay_us(15);
		GPIO_WriteBit(GPIOB, GPIO_Pin_15, (BitAction)0);
		delay_ms(30);	
}


void TIM2_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //使能TIM2定时器
	
	//配置tim2定时器
	TIM_TimeBaseStructure.TIM_Period = arr; //设置自动重装载计数计数值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //配置分频系数
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分频因子
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //设置为向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //配置到相关的定时器上面
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //配置为更新中断

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//
	NVIC_Init(&NVIC_InitStructure);//
	
//	TIM_Cmd(TIM2,ENABLE);
					 
}

float HCSR04_Get_Distance(int time)
{
	float Distance;
	Distance=(float)time/20000*340;
	return Distance;
}

void TIM2_IRQHandler(void)   //TIM2
{

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM2中断是否发生
	{		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //
	}
}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		
		EXTI_ClearITPendingBit(EXTI_Line14);
	
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==1)
		{
			TIM_SetCounter(TIM2,0);
			TIM_Cmd(TIM2, ENABLE);
		}
		else if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==0)
		{
			TIM_Cmd(TIM2, DISABLE);
			HCSR04_Distance=HCSR04_Get_Distance(TIM_GetCounter(TIM2));
		}
	}
}


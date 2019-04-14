#include "hcsr04.h"
#include "delay.h"
#include "time.h"

#define TRIG	GPIO_Pin_15//�����źţ�PB15
#define ECHO	GPIO_Pin_14//�����źţ�PB14

#define AVG_NUM 5
#define Distance_Offset 6.1f

float HCSR04_Distance=0.0;

void HCSR04_Init(void)
{
	
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	TIM2_Int_Init(30000,71);		//�����벶׽ģ�飬���г������źŲ�׽

//����gpioģ�������������ź�	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = TRIG;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	
//�����ж�ģ�飬���н����źŲ�׽�ж�	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//ʹ��b��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//ʹ�ܸ��ù���
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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʹ��TIM2��ʱ��
	
	//����tim2��ʱ��
	TIM_TimeBaseStructure.TIM_Period = arr; //�����Զ���װ�ؼ�������ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //���÷�Ƶϵ��
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷ�Ƶ����
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //����Ϊ���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //���õ���صĶ�ʱ������
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //����Ϊ�����ж�

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

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //���TIM2�ж��Ƿ���
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


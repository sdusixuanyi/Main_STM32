#include "led.h"
#include "delay.h"

//��ʼ��PB5��PE5Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{ 
	 GPIO_InitTypeDef  GPIO_InitStructure;
 	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PC�˿�ʱ��
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9| GPIO_Pin_10| GPIO_Pin_11;				 //LED0-->PC.13 �˿�����
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.8
	 GPIO_SetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);						 //PC.13 �����
}
 
void LED_ON(u8 num)
{
	switch (num)
	{
	case 0:	GPIO_ResetBits(GPIOA, GPIO_Pin_8); //LED-ON
	case 1:	GPIO_ResetBits(GPIOA, GPIO_Pin_9); //LED-ON
	case 2:	GPIO_ResetBits(GPIOA, GPIO_Pin_10); //LED-ON
	case 3:	GPIO_ResetBits(GPIOA, GPIO_Pin_11); //LED-ON
	case 4: GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);//Let all led ON
	default:delay_ms(10);
	}

}

void LED_OFF(u8 num)
{
	switch (num)
	{
	case 0:	GPIO_SetBits(GPIOA, GPIO_Pin_8); //LED-OFF
	case 1:	GPIO_SetBits(GPIOA, GPIO_Pin_9); //LED-OFF
	case 2:	GPIO_SetBits(GPIOA, GPIO_Pin_10); //LED-OFF
	case 3:	GPIO_SetBits(GPIOA, GPIO_Pin_11); //LED-OFF
	case 4: GPIO_SetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);//Let all led OFF
	default:delay_ms(10);
	}
}



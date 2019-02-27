#include "stm32f10x.h"
#include "led.h"
#include "key.h"
#include "delay.h"
#include "pwm.h"
#include "beep.h"
#include "dma.h"
#include "timer.h"
#include "hcsr04.h"


unsigned char raw_data[14] = {0};
short int translated_data[7];
float accel[3];         
float gyro[3];

void Init()
{
	SystemInit();	// 配置系统时钟为72M 	
	LED_Init();
	KEY_Init();
	delay_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	HCSR04_Init();
//	Beep_Init();
	TIM4_PWM_Init(8999,7);//PWM=72000/8/(8999+1)=500hz 
	i2c_init();
	MPU6050_init();
	dma_init(); 
	timer3_init();
}


int main(void)
{	
  	
	Init();

  while (1)
  {	
//		LED0=0;
//		delay_ms(500);
//		LED0=1;
//		delay_ms(500);	
//	TIM4_CH1_Duty(600);
//	TIM4_CH2_Duty(600);
//	TIM4_CH3_Duty(600);
//	TIM4_CH4_Duty(600);		
  }
}












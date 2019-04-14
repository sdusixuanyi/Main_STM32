#include "stm32f10x.h"
#include "led.h"
#include "key.h"
#include "delay.h"
#include "pwm.h"
//#include "beep.h"
#include "dma.h"
#include "timer.h"
#include "hcsr04.h"
#include "i2c.h"
#include "mpu6050.h"
#include "rtc.h"
#include "stmflash.h"
#include "iwdg.h"
#include "spl06.h"
#include "adc.h"
#include "ANO_DT.h"
#include "usart.h"
#include "sys.h"
//#include "24l01.h"
#include "flight_control.h"


extern long int cycle;
extern struct Status_flag status;
extern float motor_duty[4];
unsigned char raw_data[14] = {0};
short int translated_data[7];
float accel[3];         
float gyro[3];
float Power_V;
extern int dma_flag;    //dma???????1
extern struct PID_parameter PID_yaw;

void PID_parameter_debug(void);


void Init()
{
	SystemInit();	// ???????72M 	
	LED_Init();
	KEY_Init();
	delay_init();
	Adc_Init();
	uart_init (115200);
//	NRF24L01_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	HCSR04_Init();
//	Beep_Init();
	TIM4_PWM_Init(8999,7);//PWM=72000/8/(8999+1)=500hz 
	i2c_init(I2C_SOFTWARE);
	SPL06_init(SPL06_SOFTWARE);
	MPU6050_init(MPU6050_SOFTWARE);
	//dma_init(); 
	//while(RTC_Init ())
	//IWDG_Init(4,625);
	timer3_init();
}
//void Check()
//{
//	while(NRF24L01_Check())	//Checking the nrf24l01 is right or not.	
//	{
//		LED_ON(0);
//		delay_ms(200);
//		LED_OFF(0);
// 		delay_ms(200);
//	}
//	LED_OFF(1);
//}

void Setup(void)
{		
	while(1)
	{
/*******************??????*********************************/
//		if(cycle / 100 <4)
//		{
//			TIM4_CH1_Duty(DUTY_MAX);
//			TIM4_CH2_Duty(DUTY_MAX);
//			TIM4_CH3_Duty(DUTY_MAX);
//			TIM4_CH4_Duty(DUTY_MAX);
//		}
//		else if(cycle / 100 <7)
//		{
//			TIM4_CH1_Duty(DUTY_MIN);
//			TIM4_CH2_Duty(DUTY_MIN);
//			TIM4_CH3_Duty(DUTY_MIN);
//			TIM4_CH4_Duty(DUTY_MIN);
//		}
//这一段加上并且下面一行加else可设行程，去掉直接启动
/**********************??????***************************************/
		if((cycle/100)<7)
		{
			TIM4_CH1_Duty(DUTY_MIN);
			TIM4_CH2_Duty(DUTY_MIN);
			TIM4_CH3_Duty(DUTY_MIN);
			TIM4_CH4_Duty(DUTY_MIN);
		}else if((cycle/100)<9)
		{
			TIM4_CH1_Duty(520);
			TIM4_CH2_Duty(520);
			TIM4_CH3_Duty(520);
			TIM4_CH4_Duty(520);
		}
		else if((cycle/100)<10)
		{
			TIM4_CH1_Duty(DUTY_MIN);
			TIM4_CH2_Duty(DUTY_MIN);
			TIM4_CH3_Duty(DUTY_MIN);
			TIM4_CH4_Duty(DUTY_MIN);
		}else break;		
	}
}

int main(void)
{	
//	u8 tmp_buf[50],i,len; 
	
	Init();
//	Check();
//	Beep_ON(1,5);
	//Setup();
//	Beep_OFF();
	
	//******************************************************
//	while(1)
//	{
//		ANO_DT_Data_Exchange();	
//		motor_duty[0] = PID_yaw.Kd;
//	  motor_duty[1] = PID_yaw.Kd;
//	  motor_duty[2] = PID_yaw.Kd;
//	  motor_duty[3] = PID_yaw.Kd;
//		TIM4_CH1_Duty(motor_duty[0]);
//		TIM4_CH2_Duty(motor_duty[1]);
//		TIM4_CH3_Duty(motor_duty[2]);
//		TIM4_CH4_Duty(motor_duty[3]);
//	}
	
	//******************************************************

	
  while (1)
  {

    //PID_parameter_debug();		
    HCSR04_Run();
		TIM4_CH1_Duty(motor_duty[0]);
		TIM4_CH2_Duty(motor_duty[1]);
		TIM4_CH3_Duty(motor_duty[2]);
		TIM4_CH4_Duty(motor_duty[3]);
		//ANO_DT_Data_Exchange();	
    flight_control(COMMON_READ, MPU6050_SOFTWARE);
    SPL06_height_process(SPL06_SOFTWARE);			
  }
}


void PID_parameter_debug(void)
{
	static int restart = 1;
	  if(status.data_send == 1)
		{
			ANO_DT_Data_Exchange();	
			status.data_send = 0;
		}
		if(status.attitude_process == 1 && PID_yaw.Kp == 2)
		{
			if(restart == 1)
			{
				motor_duty[0] = STA_DUTY;
	      motor_duty[1] = STA_DUTY;
	      motor_duty[2] = STA_DUTY;
	      motor_duty[3] = STA_DUTY;
				
				restart = 0;
			}
			
			if(PID_yaw.Ki == 1)    		 flight_control_debug(DEBUG_MODE_PITCH, 0, COMMON_READ, MPU6050_SOFTWARE);
			else if(PID_yaw.Ki == 0)   flight_control_debug(DEBUG_MODE_PITCH, 45, COMMON_READ, MPU6050_SOFTWARE);
			
			status.attitude_process = 0;
		}
		if(PID_yaw.Kp == 1 || PID_yaw.Kp == 0)
		{
			  motor_duty[0] = DUTY_MIN;
	      motor_duty[1] = DUTY_MIN;
	      motor_duty[2] = DUTY_MIN;
	      motor_duty[3] = DUTY_MIN;
			
			  restart = 1;
		}
}









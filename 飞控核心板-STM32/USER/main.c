//#include "stm32f10x.h"
//#include "led.h"
//#include "key.h"
//#include "delay.h"
//#include "pwm.h"
//#include "beep.h"
//#include "dma.h"
//#include "timer.h"
//#include "hcsr04.h"
//#include "i2c.h"
//#include "mpu6050.h"
//#include "rtc.h"
//#include "stmflash.h"
//#include "iwdg.h"
//#include "spl06.h"
//#include "adc.h"
//#include "ANO_DT.h"
//#include "usart.h"
//#include "sys.h"
//#include "24l01.h"

#include "headfile.h"

unsigned char raw_data[14] = {0};
short int translated_data[7];
float accel[3];         
float gyro[3];
float Power_V;
extern int dma_flag;    //dma传输完成后会置1
//u8 TEXT_Buffer[10] = { "0123456789" };

extern struct Sys_flag SYSFLAG;
void ExecuteTask(void);

void Init()
{
	SystemInit();	// 配置系统时钟为72M 	
	LED_Init();
	KEY_Init();
	delay_init();
	Adc_Init();
	uart_init (115200);
	NRF24L01_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	HCSR04_Init();
//	Beep_Init();
	//TIM4_PWM_Init(8999,7);//PWM=72000/8/(8999+1)=500hz 
	i2c_init(I2C_SOFTWARE);
	//SPL06_init(SPL06_SOFTWARE);
	MPU6050_init(MPU6050_DMP);
	//dma_init(); 
	//while(RTC_Init ())
	//IWDG_Init(4,625);
	//timer3_init();
}
void Check()
{
	while(NRF24L01_Check())	//Checking the nrf24l01 is right or not.	
	{
		LED_ON(0);
		delay_ms(200);
		LED_OFF(0);
 		delay_ms(200);
	}
	LED_OFF(1);
}


int main(void)
{	
  	
	Init();
	Check();
  while (1)
  {	

		ANO_DT_Data_Exchange();
		Power_V=Get_Adc(1)*330*4.49/4096;
		
		ExecuteTask();       //查询并SYSFLAG并执行相应任务

    //SPL06_height_process(SPL06_SOFTWARE);			
  }
}

void ExecuteTask(void)
{
	if(SYSFLAG.AttitudeProcessPermission == 1)        //判断是否执行姿态解算
	{
		flight_control(NDMA_READ, MPU6050_DMP);
		SYSFLAG.AttitudeProcessPermission = 0;
	}
}










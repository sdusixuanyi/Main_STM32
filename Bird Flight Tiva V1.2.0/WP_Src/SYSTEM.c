#include "Headfile.h"
#include "SYSTEM.h"



void HardWave_Init(void)
{
  ConfigureUART0();//串口0初始化，山外地面站、锡月地面站
  Version_Declaration();//版本说明
  Butterworth_Parameter_Init();//滤波器参数初始化
  PPM_Init();//PPM解析初始化
  Init_PWM();//PWM初始化
  OLED_Init();//OLED显示屏初始化
  Bling_Init();//LED状态指示灯初始化
  Quad_Start_Bling();//开机LED预显示
  Key_Init();//板载按键初始化
  RC_Calibration_Trigger();//遥控器行程标定检查
  Horizontal_Calibration_Init();//机架水平标定初始化
  PID_Paramter_Init_With_Flash();//PID参数初始化
  Init_I2C();//硬件I2C初始化
  Init_MPU6050();//MPU6050陀螺仪、加速度计配置初始化
  IST8310_Init();//IST8310磁力计配置初始化
  SPL06_Init();//SPL06气压计配置初始化
  Parameter_Init();//加速度计、磁力计校准参数初始化
  WP_Quad_Init();//根据观测传感器加速度计、磁力计对姿态四元数初始化
  ConfigureUART1();//串口1初始化，匿名地面站
  ConfigureUART3();//串口3初始化，OPENMV、树莓派视觉模组、SDK模式串口
  Set_GPS_USART();//串口2初始化，使用UBLOX协议配置GPS模块
  ConfigureUART7();//串口7初始化.超声波模块/TOF解析串口	
  ConfigureUART6();//串口6初始化，光流模块LC306数据解析
  ADC_Init();//ADC初始化，检测外部电池电压
  SDK_Init();//SDK模式初始化
  Time_init();//飞控调度定时器初始化
  delay_ms(200);//初始化延时	
}


void ESC_HardWave_Init()//只初始化校准电调的必要资源
{
  OLED_Init_Fast();
  PPM_Init();
  Init_PWM();
  delay_ms(100);
  WriteFlashParameter(ESC_CALIBRATION_FLAG,0,&Table_Parameter);//写零避免下次上电再次进入
  while(1)
  {
    ESC_Calibration();
    LCD_clear_L(0,0);LCD_clear_L(0,1);LCD_P8x16Str(0,0,"Please Move Thr");
    LCD_clear_L(0,2);LCD_clear_L(0,3);LCD_P8x16Str(0,2,"Down When ESC");
    LCD_clear_L(0,4);LCD_clear_L(0,5);LCD_P8x16Str(0,4,"Beep Beep");
    LCD_P6x8Str(80,4,"Thr:");
    write_6_8_number(80,5,PPM_Databuf[2]);
    LCD_clear_L(0,6);LCD_P6x8Str(0,6,"Repower When Set Up");
  }
}

void WP_Init(void)
{
  System_Start_Init();//系统启动初始化
  ReadFlashParameterOne(ESC_CALIBRATION_FLAG,&ESC_Calibration_Flag);
  if(ESC_Calibration_Flag==1)
  {
    ESC_HardWave_Init();//只初始化校准电调的必要资源 
  }
  else
  {
    HardWave_Init();//飞控板内部资源、相关外设初始化
  }
}


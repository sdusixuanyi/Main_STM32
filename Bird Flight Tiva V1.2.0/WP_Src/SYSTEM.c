#include "Headfile.h"
#include "SYSTEM.h"



void HardWave_Init(void)
{
  ConfigureUART0();//����0��ʼ����ɽ�����վ�����µ���վ
  Version_Declaration();//�汾˵��
  Butterworth_Parameter_Init();//�˲���������ʼ��
  PPM_Init();//PPM������ʼ��
  Init_PWM();//PWM��ʼ��
  OLED_Init();//OLED��ʾ����ʼ��
  Bling_Init();//LED״ָ̬ʾ�Ƴ�ʼ��
  Quad_Start_Bling();//����LEDԤ��ʾ
  Key_Init();//���ذ�����ʼ��
  RC_Calibration_Trigger();//ң�����г̱궨���
  Horizontal_Calibration_Init();//����ˮƽ�궨��ʼ��
  PID_Paramter_Init_With_Flash();//PID������ʼ��
  Init_I2C();//Ӳ��I2C��ʼ��
  Init_MPU6050();//MPU6050�����ǡ����ٶȼ����ó�ʼ��
  IST8310_Init();//IST8310���������ó�ʼ��
  SPL06_Init();//SPL06��ѹ�����ó�ʼ��
  Parameter_Init();//���ٶȼơ�������У׼������ʼ��
  WP_Quad_Init();//���ݹ۲⴫�������ٶȼơ������ƶ���̬��Ԫ����ʼ��
  ConfigureUART1();//����1��ʼ������������վ
  ConfigureUART3();//����3��ʼ����OPENMV����ݮ���Ӿ�ģ�顢SDKģʽ����
  Set_GPS_USART();//����2��ʼ����ʹ��UBLOXЭ������GPSģ��
  ConfigureUART7();//����7��ʼ��.������ģ��/TOF��������	
  ConfigureUART6();//����6��ʼ��������ģ��LC306���ݽ���
  ADC_Init();//ADC��ʼ��������ⲿ��ص�ѹ
  SDK_Init();//SDKģʽ��ʼ��
  Time_init();//�ɿص��ȶ�ʱ����ʼ��
  delay_ms(200);//��ʼ����ʱ	
}


void ESC_HardWave_Init()//ֻ��ʼ��У׼����ı�Ҫ��Դ
{
  OLED_Init_Fast();
  PPM_Init();
  Init_PWM();
  delay_ms(100);
  WriteFlashParameter(ESC_CALIBRATION_FLAG,0,&Table_Parameter);//д������´��ϵ��ٴν���
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
  System_Start_Init();//ϵͳ������ʼ��
  ReadFlashParameterOne(ESC_CALIBRATION_FLAG,&ESC_Calibration_Flag);
  if(ESC_Calibration_Flag==1)
  {
    ESC_HardWave_Init();//ֻ��ʼ��У׼����ı�Ҫ��Դ 
  }
  else
  {
    HardWave_Init();//�ɿذ��ڲ���Դ����������ʼ��
  }
}


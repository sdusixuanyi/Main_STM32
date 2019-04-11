#include "Headfile.h"
int main(void)
{
  WP_Init();//芯片资源、飞控外设初始化
  while(1)//主循环
  {			
    Key_Scan(Key_Right_Release);//按键扫描
    QuadShow();//OLED显示
    ANO_SEND_StateMachine();//ANO地面站发送
    Accel_Calibartion();//加速度计6面校准
    Mag_Calibartion_LS(&WP_Sensor.mag_raw,Circle_Angle);//磁力计椭球校准
//    RC_Calibration_Check(PPM_Databuf);//遥控器行程校准
	Save_Or_Reset_PID_Parameter();//运用地面站，修改控制参数
  }
}



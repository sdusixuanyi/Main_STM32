#include "Headfile.h"
int main(void)
{
  WP_Init();//оƬ��Դ���ɿ������ʼ��
  while(1)//��ѭ��
  {			
    Key_Scan(Key_Right_Release);//����ɨ��
    QuadShow();//OLED��ʾ
    ANO_SEND_StateMachine();//ANO����վ����
    Accel_Calibartion();//���ٶȼ�6��У׼
    Mag_Calibartion_LS(&WP_Sensor.mag_raw,Circle_Angle);//����������У׼
//    RC_Calibration_Check(PPM_Databuf);//ң�����г�У׼
	Save_Or_Reset_PID_Parameter();//���õ���վ���޸Ŀ��Ʋ���
  }
}



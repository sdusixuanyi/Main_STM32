//#include "headfile.h"
#include "sys.h"
#include "debug_para.h"
#include "ano_dt.h"
/*
�Զ�������������
���룺��Ҫ������Kp��ַ����Ӧ��������������
����0��δ��⵽�ֲ����ţ�ֱ�ӷ���
1:���δ�����״̬
2:���δ�������״̬
3:���δ�����Եȷ���״̬
4:����״̬�Ƚϸ��ӣ��޷����й���
*/
u8 Auto_para(float *Kp, float Wave_now)
{
	static u16 Wave_max[5] = {0};
	static u16 Wave_min[5] = {0};
	static u8  Max_cnt = 0;
	static u8  Min_cnt = 0;
	static u8  Dir_flag = 1;
	static u16 Wave_last = 0;
	static u16 _cnt=0;
	u8 i;u16 Wave;
	int Max_sum, Min_sum;
	Wave=(u16)(Wave_now*1000);
	//���ȸ�������Ҫ�Ĳ����洢��
	if (Dir_flag == 1)//flag=1�������ݲ���������������Ҫ��ȡ�ֲ����
	{
		if (Wave_last < Wave)
		{
			Wave_last = Wave;//�������ݷ���
			_cnt=0;
			return 0;
		}
		else if(_cnt<10)
		{
			_cnt++;
		}else
		{
			Wave_max[Max_cnt] = Wave_last;
			Wave_last = Wave;
			Max_cnt++;
			_cnt=0;
			Dir_flag = 0;
		}
	}else if (Dir_flag == 0)//flag=0;�������ݲ��������½�����Ҫ��ȡ�ֲ���С
	{
		if (Wave_last > Wave)
		{
			Wave_last = Wave;
			_cnt=0;
			return 0;
		}
		else if(_cnt<10)
		{
			_cnt++;
		}else
		{
			Wave_min[Min_cnt] = Wave_last;
			Wave_last = Wave;
			Min_cnt++;
			_cnt=0;
			Dir_flag = 1;
		}
	}
	if (Min_cnt == 6 && Max_cnt == 6)//��ֵ������������Ҫ����PID����֮�����㿪ʼ�ٴμ���
	{
		Min_cnt = 0;
		Max_cnt = 0;
		for (i = 0; i < 4; i++)
		{
			Max_sum = Max_sum+(Wave_max[i + 1] - Wave_max[i]);
			Min_sum = Min_sum+(Wave_min[i + 1] - Wave_min[i]);
			Wave_max[i] = 0;
			Wave_min[i] = 0;
		}
		Wave_max[4] = 0; Wave_min[4] = 0;//���֮ǰ�����ݣ�׼�������´β�������
		if (Max_sum > Max_scale && Min_sum < Min_scale)//�ж����ݲ��δ�����״̬���˿�Ӧ����СKp��ֵ
		{
			*Kp = *Kp - 0.00005;
//			f.send_pid1=1;
//			f.send_pid2=1;
//			f.send_pid3=1;
			return 1;
		}
		else if (Max_sum < Min_scale && Min_sum > Max_scale)//�ж����ݲ��δ�������״̬���˿�Ӧ������Kp��ֵ
		{
			*Kp = *Kp + 0.00005;
//			f.send_pid1=1;
//			f.send_pid2=1;
//			f.send_pid3=1;
			return 2;
		}
		else if (Max_sum<Max_scale&&Max_sum>Min_scale&&Min_sum<Max_scale&&Min_sum>Min_scale)
		{
			////write_flash(Kp*0.7)	//���˿̵õ���Kp��ֵ��0.7�Ժ�д��flash����Ϊ���յ�flashʹ��
//			f.send_pid1=1;
//			f.send_pid2=1;
//			f.send_pid3=1;
			return 3;
		}
		else
		{
			return 4;
		}
	}
	return 4;
}



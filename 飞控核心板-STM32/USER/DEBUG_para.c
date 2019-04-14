//#include "headfile.h"
#include "sys.h"
#include "debug_para.h"
#include "ano_dt.h"
/*
自动参数调整函数
传入：想要调整的Kp地址；对应反馈参量的数据
返回0：未检测到局部最优，直接返回
1:波形处于震荡状态
2:波形处于收敛状态
3:波形处于相对等幅震荡状态
4:波形状态比较复杂，无法进行归类
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
	//首先更新所需要的参数存储区
	if (Dir_flag == 1)//flag=1代表数据波形正在增长，需要求取局部最大
	{
		if (Wave_last < Wave)
		{
			Wave_last = Wave;//更新数据返回
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
	}else if (Dir_flag == 0)//flag=0;代表数据波形正在下降，需要求取局部最小
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
	if (Min_cnt == 6 && Max_cnt == 6)//最值缓存区满，需要更新PID参数之后清零开始再次计数
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
		Wave_max[4] = 0; Wave_min[4] = 0;//清除之前的数据，准备进行下次参数调整
		if (Max_sum > Max_scale && Min_sum < Min_scale)//判断数据波形处于震荡状态，此刻应当减小Kp的值
		{
			*Kp = *Kp - 0.00005;
//			f.send_pid1=1;
//			f.send_pid2=1;
//			f.send_pid3=1;
			return 1;
		}
		else if (Max_sum < Min_scale && Min_sum > Max_scale)//判断数据波形处于收敛状态，此刻应当增大Kp的值
		{
			*Kp = *Kp + 0.00005;
//			f.send_pid1=1;
//			f.send_pid2=1;
//			f.send_pid3=1;
			return 2;
		}
		else if (Max_sum<Max_scale&&Max_sum>Min_scale&&Min_sum<Max_scale&&Min_sum>Min_scale)
		{
			////write_flash(Kp*0.7)	//将此刻得到的Kp数值乘0.7以后写入flash，作为最终的flash使用
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



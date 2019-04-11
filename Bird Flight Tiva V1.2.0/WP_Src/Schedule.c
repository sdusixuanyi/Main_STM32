#include "Headfile.h"
#include "Time.h"
#include "Schedule.h"

void Schedule_init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);//��ʱ��1ʹ��				
  TimerConfigure(TIMER1_BASE,TIMER_CFG_PERIODIC_UP);//32λ���ڶ�ʱ��
  TimerLoadSet(TIMER1_BASE,TIMER_A,SysCtlClockGet()/100);//�趨װ��ֵ,��80M/100��*1/80M=10ms				
  IntEnable(INT_TIMER1A);//��ʱ��1�ж�ʹ��				
  TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT); //�ж����, ����ģʽ;
  TimerIntRegister(TIMER1_BASE,TIMER_A,TIMER1A_Handler);//�жϺ���ע��
  //IntMasterEnable();			
  TimerEnable(TIMER1_BASE,TIMER_A); //��ʱ��ʹ�ܿ�ʼ����	
  //IntPriorityGroupingSet(0);
  IntPrioritySet(INT_TIMER1A,USER_INT0);
}



//#define Hour         3
//#define Minute       2
//#define Second       1
//#define MicroSecond  0
//uint16 Time_Sys[4]={0};
//uint16 Microsecond_Cnt=0;
//uint32 TIME_ISR_CNT=0,LAST_TIME_ISR_CNT=0;
void TIMER1A_Handler(void)//�жϺ���
{
  //LAST_TIME_ISR_CNT=TIME_ISR_CNT;
  //TIME_ISR_CNT++;//ÿ10ms�Լ�
  //	Microsecond_Cnt++;
  //	if(Microsecond_Cnt>=100)//1s
  //	{
  //		Microsecond_Cnt=0;
  //		Time_Sys[Second]++;
  //		if(Time_Sys[Second]>=60)//1min
  //		{
  //			Time_Sys[Second]=0;
  //			Time_Sys[Minute]++;
  //			if(Time_Sys[Minute]>=60)//1hour
  //			{
  //				Time_Sys[Minute]=0;
  //				Time_Sys[Hour]++;
  //			}
  //		}
  //	}
  TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT);  	
}

void Test_Period(Testime *Time_Lab)
{
  Time_Lab->Last_Time=Time_Lab->Now_Time;
  Time_Lab->Now_Time=micros()/1000.0f;//(10000*TIME_ISR_CNT+TimerValueGet(TIMER1_BASE,TIMER_A)/80.0f)/1000.0f;//��λms
  Time_Lab->Time_Delta=(Time_Lab->Now_Time-Time_Lab->Last_Time);
  Time_Lab->Time_Delta_INT=(uint16)(Time_Lab->Time_Delta);
}

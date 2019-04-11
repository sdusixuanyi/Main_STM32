#include "Headfile.h"
#include "Time.h"
#include "Schedule.h"

void Schedule_init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);//定时器1使能				
  TimerConfigure(TIMER1_BASE,TIMER_CFG_PERIODIC_UP);//32位周期定时器
  TimerLoadSet(TIMER1_BASE,TIMER_A,SysCtlClockGet()/100);//设定装载值,（80M/100）*1/80M=10ms				
  IntEnable(INT_TIMER1A);//定时器1中断使能				
  TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT); //中断输出, 设置模式;
  TimerIntRegister(TIMER1_BASE,TIMER_A,TIMER1A_Handler);//中断函数注册
  //IntMasterEnable();			
  TimerEnable(TIMER1_BASE,TIMER_A); //定时器使能开始计数	
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
void TIMER1A_Handler(void)//中断函数
{
  //LAST_TIME_ISR_CNT=TIME_ISR_CNT;
  //TIME_ISR_CNT++;//每10ms自加
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
  Time_Lab->Now_Time=micros()/1000.0f;//(10000*TIME_ISR_CNT+TimerValueGet(TIMER1_BASE,TIMER_A)/80.0f)/1000.0f;//单位ms
  Time_Lab->Time_Delta=(Time_Lab->Now_Time-Time_Lab->Last_Time);
  Time_Lab->Time_Delta_INT=(uint16)(Time_Lab->Time_Delta);
}

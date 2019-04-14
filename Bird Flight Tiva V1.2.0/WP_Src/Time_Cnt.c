#include "Headfile.h"
#include "Time_Cnt.h"
void Time_init(void)//系统调度定时器初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);//定时器0使能				
  TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);//32位周期定时器				
  TimerLoadSet(TIMER0_BASE,TIMER_A,SysCtlClockGet()/200);//设定装载值,（80M/200）*1/80M=5ms				
  IntEnable(INT_TIMER0A);//总中断使能				
  TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT); //中断输出, 设置模式;			
  TimerIntRegister(TIMER0_BASE,TIMER_A,TIMER0A_Handler);//中断函数注册
  IntMasterEnable();			
  TimerEnable(TIMER0_BASE,TIMER_A); //定时器使能开始计数
  IntPrioritySet(INT_TIMER0A,USER_INT7);
}

Sensor WP_Sensor;
Testime Time0_Delta;
void TIMER0A_Handler(void)//系统调度中断函数
{
  Test_Period(&Time0_Delta);
//  Remote_Control();//遥控器数据解析
  Get_Status_Feedback();//获取姿态数据、水平与竖直方向惯导数据
//  US_100_Statemachine();//超声波传感器状态机更新
//  Optflow_Statemachine();//光流状态机，初始化时存在光流外设
//  SDK_Data_Prase();//SDK数据解析
//  GPS_Data_Prase();//GPS数据解析
//  KalmanFilter_Horizontal();//水平位置GPS双观测量Kalman滤波融合	
  CarryPilot_Control();//总控制器
//  Calibration_Check_All();//总校准函数
  Bling_Working(Bling_Mode);//状态指示灯运行
  TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
}


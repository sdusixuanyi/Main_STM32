/*----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于无名科创团队，无名科创团队将飞控程序源码提供给购买者，
*               购买者要为无名科创团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载， 
*               更不能以此销售牟利，如发现上述行为，无名科创团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名科创开源飞控QQ群：540707961
*               CSDN博客：http://blog.csdn.net/u011992534
*               优酷ID：NamelessCotrun无名小哥
*               B站教学视频：https://space.bilibili.com/67803559/#/video
*               客户使用心得、改进意见征集贴：http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               百度贴吧:无名科创开源飞控
*               修改日期:2018/8/30
*               版本：慧飞者——V1.0.1
*               版权所有，盗版必究。
*               Copyright(C) 武汉科技大学无名科创团队 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "Bling.h"

/***************************************************
函数名: void Bling_Init(void)
说明:	LED状态指示灯初始化
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void Bling_Init()
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; 
  HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x000000FF;
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2 |GPIO_PIN_3);
  GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_2);
}


Bling_Light Light_1,Light_2,Light_3,Light_4;
uint16_t Bling_Mode=0;
/***************************************************
函数名: void Bling_Set(Bling_Light *Light,
uint32_t Continue_time,//持续时间
uint16_t Period,//周期100ms~1000ms
float Percent,//0~100%
uint16_t  Cnt,
GPIO_TypeDef* Port,
uint16_t Pin
,uint8_t Flag)
说明:	状态指示灯设置函数
入口:	时间、周期、占空比、端口等
出口:	无
备注:	程序初始化后、始终运行
****************************************************/
void Bling_Set(Bling_Light *Light,
               uint32_t Continue_time,//持续时间
               uint16_t Period,//周期100ms~1000ms
               float Percent,//0~100%
               uint16_t  Cnt,
               uint32_t Port,
               uint16_t Pin
                 ,uint8_t Flag)
{
  Light->Bling_Contiune_Time=(uint16_t)(Continue_time/5);//持续时间
  Light->Bling_Period=Period;//周期
  Light->Bling_Percent=Percent;//占空比
  Light->Bling_Cnt=Cnt;
  Light->Port=Port;//端口
  Light->Pin=Pin;//引脚
  Light->Endless_Flag=Flag;//无尽模式
}

/***************************************************
函数名: void Bling_Process(Bling_Light *Light)//闪烁运行线程
说明:	状态指示灯实现
入口:	状态灯结构体
出口:	无
备注:	程序初始化后、始终运行
****************************************************/
void Bling_Process(Bling_Light *Light)//闪烁运行线程
{
  if(Light->Bling_Contiune_Time>=1)  Light->Bling_Contiune_Time--;
  else GPIOPinWrite(Light->Port,Light->Pin,Light->Pin);//置高,亮
  if(Light->Bling_Contiune_Time!=0//总时间未清0
     ||Light->Endless_Flag==1)//判断无尽模式是否开启
  {
    Light->Bling_Cnt++;
    if(5*Light->Bling_Cnt>=Light->Bling_Period) Light->Bling_Cnt=0;//计满清零
    if(5*Light->Bling_Cnt<=Light->Bling_Period*Light->Bling_Percent)
      GPIOPinWrite(Light->Port,Light->Pin,Light->Pin);//置高，亮
    else GPIOPinWrite(Light->Port,Light->Pin,0);//置低，灭
  }
}


void GPIO_SetBits(Bling_Light *Light)
{
  GPIOPinWrite(Light->Port,Light->Pin,Light->Pin);//置高，亮
}

void GPIO_ResetBits(Bling_Light *Light)
{
  GPIOPinWrite(Light->Port,Light->Pin,0);//置低，灭
}
/***************************************************
函数名: Bling_Working(uint16 bling_mode)
说明:	状态指示灯状态机
入口:	当前模式
出口:	无
备注:	程序初始化后、始终运行
****************************************************/
void Bling_Working(uint16 bling_mode)
{
  if(bling_mode==0)//全灭
  {
    Bling_Process(&Light_1);
    Bling_Process(&Light_2);
    Bling_Process(&Light_3);
  }
  else if(bling_mode==1)//加速度计6面校准模式
  {
    if(flight_direction==0)//第一面校准准备
    {
      Bling_Process(&Light_1);
      GPIO_SetBits(&Light_2);
      GPIO_SetBits(&Light_3);
    }
    else if(flight_direction==1)//第二面校准准备
    {
      Bling_Process(&Light_2);
      GPIO_SetBits(&Light_1);
      GPIO_SetBits(&Light_3);
    }
    else if(flight_direction==2)//第三面校准准备
    {
      Bling_Process(&Light_1);
      Bling_Process(&Light_2);
      GPIO_SetBits(&Light_3);
    }
    else if(flight_direction==3)//第四面校准准备
    {
      Bling_Process(&Light_3);
      GPIO_SetBits(&Light_1);
      GPIO_SetBits(&Light_2);
    }
    else if(flight_direction==4)//第五面校准准备
    {
      Bling_Process(&Light_1);
      Bling_Process(&Light_3);
      GPIO_SetBits(&Light_2);
    }
    else if(flight_direction==5)//第六面校准准备
    {
      GPIO_SetBits(&Light_1);
      Bling_Process(&Light_2);
      Bling_Process(&Light_3);
    }
    else
    {
      Bling_Process(&Light_1);
      Bling_Process(&Light_2);
      Bling_Process(&Light_3);
    }
  }
  else if(bling_mode==2)//磁力计校准模式
  {
    if(Mag_Calibration_Mode==0)//水平面校准
    {
      Bling_Process(&Light_1);
      GPIO_SetBits(&Light_2);
      GPIO_SetBits(&Light_3);
    }
    else if(Mag_Calibration_Mode==1)////竖直平面校准
    {
      GPIO_SetBits(&Light_1);
      Bling_Process(&Light_2);
      GPIO_SetBits(&Light_3);
    }
    else if(Mag_Calibration_Mode==2)////竖直平面校准
    {
      GPIO_SetBits(&Light_1);
      GPIO_SetBits(&Light_2);
      Bling_Process(&Light_3);
    }
    else
    {
      Bling_Process(&Light_1);
      Bling_Process(&Light_2);
      Bling_Process(&Light_3);
    }
  }
  else if(bling_mode==3)//全灭
  {
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,GPIO_PIN_3);
  }
  
  if(((Last_GPS_Sate_Num!=GPS_Sate_Num)&&GPS_Sate_Num<=5)
     ||Filter_Defeated_Flag==1)//满足会一直长亮
    Bling_Set(&Light_4,1000,1000,0.5,0,GPIO_PORTC_BASE,GPIO_PIN_2,1);
  else if((Last_GPS_Sate_Num!=GPS_Sate_Num)&&GPS_Sate_Num<=9)
    Bling_Set(&Light_4,2000,2000,0.5,0,GPIO_PORTC_BASE,GPIO_PIN_2,1);
  else if((Last_GPS_Sate_Num!=GPS_Sate_Num)&&GPS_Sate_Num>=10)
    Bling_Set(&Light_4,2000,5000,0.5,0,GPIO_PORTC_BASE,GPIO_PIN_2,1);
  Bling_Process(&Light_4);
  
  
}


/***************************************************
函数名: void Quad_Start_Bling(void)
说明:	LED初始化后开机闪烁
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void Quad_Start_Bling()
{
  Bling_Set(&Light_1,2000,200,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);
  Bling_Set(&Light_2,2000,500,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);
  Bling_Set(&Light_3,2000,800,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,0);
  Bling_Set(&Light_4,2000,1000,0.5,0,GPIO_PORTC_BASE,GPIO_PIN_2,1);
}


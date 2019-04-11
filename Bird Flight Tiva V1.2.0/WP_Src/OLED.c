#include "Headfile.h"
#include "Time.h"
#include "string.h"
#include "oledfont.h"
#include "oled.h"
#include "stdlib.h"


//#define  OLED_WORK_MODE_I2C  1//OLED使用I2C模式驱动，当采用SPI模式时，注释本行即可

#define XLevelL  	0x00
#define XLevelH  	0x10
#define XLevel    ((XLevelH&0x0F)*16+XLevelL)
#define Max_Column 	128
#define Max_Row  	64
#define Brightness 	0xCF
#define X_WIDTH 	128
#define Y_WIDTH 	64
// ------------------------------------------------------------
// IO口模拟I2C通信，SCL接PC5、SDA接PC4
// ------------------------------------------------------------
void OLED_GPIO_Init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIO_OLED);// Enable the GPIO port that is used for the on-board LED.
  GPIOPinTypeGPIOOutput(GPIO_OLED, GPIO_PIN_5);
  GPIOPinTypeGPIOOutput(GPIO_OLED, GPIO_PIN_4);
  Delay_Ms(2);
  GPIOPinWrite(GPIO_OLED, GPIO_PIN_5, GPIO_PIN_5);//设置高
  GPIOPinWrite(GPIO_OLED, GPIO_PIN_4, GPIO_PIN_4);//设置高
}


void SDA_OUT(void)
{
  GPIOPinTypeGPIOOutput(GPIO_OLED, GPIO_PIN_4);// Enable the GPIO pins for the LED (PF2 & PF3).
}


void SDA_IN(void)
{
  GPIOPinTypeGPIOInput(GPIO_OLED, GPIO_PIN_4);// Enable the GPIO pins for the LED (PF2 & PF3).
}

/***************************
函数名:	void IIC_Start(void)
说明:	产生IIC起始信号
入口:	无
出口:	无
备注:	无
****************************/
void OLED_IIC_Start(void)
{
  SDA_OUT();		// SDA线输出
  IIC_SDA_H;
  IIC_SCL_H;
  //Delay_Us(4);
  IIC_SDA_L;
  //Delay_Us(1);
  IIC_SCL_L;		// 钳住I2C总线,准备发送或接收数据 
}

/***************************
函数名:	void IIC_Stop(void)
说明:	产生IIC停止信号
入口:	无
出口:	无
备注:	无
****************************/
void OLED_IIC_Stop(void)
{
  SDA_OUT();			// SDA线输出
  IIC_SDA_L;
  //Delay_Us(4);
  IIC_SCL_H;
  //Delay_Us(1);
  IIC_SDA_H;			// 发送I2C总线结束信号
  //Delay_Us(5);		// 再次启动需要4.7us
}

/**********************************
函数名:	uint8_t IIC_Wait_Ack(void)
说明:	等待应答信号到来
入口:	无
出口:	uint8_t 1,接收应答失败
0,接收应答成功
备注:	无
**********************************/
uint8_t IIC_Wait_Ack(void)
{
  uint8_t ucErrTime = 0;
  SDA_IN();					// SDA设置为输入  
  IIC_SDA_H;
  //Delay_Us(1);		
  IIC_SCL_H;
  //Delay_Us(1);	 
  while (READ_SDA)			// SDA为高,等待IIC器件拉低
  {
    ucErrTime++;
    if (ucErrTime > 250)	//	40*250=1ms未答复,IIC发出停止信号
    {
      OLED_IIC_Stop();
      return 1;
    }
  }
  IIC_SCL_L;
  return 0;
}

/**************************
函数名:	void IIC_Ack(void)
说明:	产生ACK应答
入口:	无
出口:	无
备注:	无
**************************/
void IIC_Ack(void)
{
  IIC_SCL_L;
  SDA_OUT();
  IIC_SDA_L;
  //Delay_Us(1);
  IIC_SCL_H;
  //Delay_Us(1);
  IIC_SCL_L;
}


/***************************
函数名:	void IIC_NAck(void)
说明:	不产生ACK应答
入口:	无
出口:	无
备注:	无
***************************/
void IIC_NAck(void)
{
  IIC_SCL_L;
  SDA_OUT();
  IIC_SDA_H;
  //Delay_Us(1);
  IIC_SCL_H;
  //Delay_Us(1);
  IIC_SCL_L;
}

/**************************************
函数名:	void IIC_Send_Byte(uint8_t txd)
说明:	IIC发送一个字节
入口:	uint8_t	txd
出口:	无
备注:	无
**************************************/
void Write_IIC_Byte(uint8_t txd)
{
  uint8_t t;
  
  SDA_OUT();
  IIC_SCL_L;				// 拉低时钟开始数据传输
  for (t = 0; t < 8; t++)
  {
    if ((txd&0x80) >> 7)
    {
      IIC_SDA_H;
    }
    else
    {
      IIC_SDA_L;
    }
    txd <<= 1;
    //Delay_Us(1);
    IIC_SCL_H;
    //Delay_Us(1); 
    IIC_SCL_L;
  }
  IIC_Wait_Ack();
}

/*********************OLED写数据************************************/ 
void OLED_WrDat(unsigned char IIC_Data)
{
  OLED_IIC_Start();
  Write_IIC_Byte(0x78);
  Write_IIC_Byte(0x40);			//write data
  Write_IIC_Byte(IIC_Data);
  OLED_IIC_Stop();
}
/*********************OLED写命令************************************/
void OLED_WrCmd(unsigned char IIC_Command)
{
  OLED_IIC_Start();
  Write_IIC_Byte(0x78);            //Slave address,SA0=0
  Write_IIC_Byte(0x00);			//write command
  Write_IIC_Byte(IIC_Command);
  OLED_IIC_Stop();
}
/*********************OLED 设置坐标************************************/
void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 
  OLED_WrCmd(0xb0+y);
  OLED_WrCmd(((x&0xf0)>>4)|0x10);
  OLED_WrCmd((x&0x0f)|0x01);
} 
/*********************OLED全屏************************************/
void OLED_Fill(unsigned char bmp_dat) 
{
  unsigned char y,x;
  for(y=0;y<8;y++)
  {
    OLED_WrCmd(0xb0+y);
    OLED_WrCmd(0x01);
    OLED_WrCmd(0x10);
    for(x=0;x<X_WIDTH;x++)
      OLED_WrDat(bmp_dat);
  }
}
/*********************OLED复位************************************/
void OLED_CLS(void)
{
  unsigned char y,x;
  for(y=0;y<8;y++)
  {
    OLED_WrCmd(0xb0+y);
    OLED_WrCmd(0x01);
    OLED_WrCmd(0x10);
    for(x=0;x<X_WIDTH;x++)
      OLED_WrDat(0);
  }
}
/*********************OLED初始化************************************/
void OLED_Init_I2C(void)
{
  Delay_Ms(100);//初始化之前的延时很重要！
  OLED_WrCmd(0xae);//--turn off oled panel
  OLED_WrCmd(0x00);//---set low column address
  OLED_WrCmd(0x10);//---set high column address
  OLED_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
  OLED_WrCmd(0x81);//--set contrast control register
  OLED_WrCmd(Brightness); // Set SEG Output Current Brightness
  OLED_WrCmd(0xa0);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
  OLED_WrCmd(0xc0);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
  OLED_WrCmd(0xa6);//--set normal display
  OLED_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
  OLED_WrCmd(0x3f);//--1/64 duty
  OLED_WrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
  OLED_WrCmd(0x00);//-not offset
  OLED_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
  OLED_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
  OLED_WrCmd(0xd9);//--set pre-charge period
  OLED_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  OLED_WrCmd(0xda);//--set com pins hardware configuration
  OLED_WrCmd(0x12);
  OLED_WrCmd(0xdb);//--set vcomh
  OLED_WrCmd(0x40);//Set VCOM Deselect Level
  OLED_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
  OLED_WrCmd(0x02);//
  OLED_WrCmd(0x8d);//--set Charge Pump enable/disable
  OLED_WrCmd(0x14);//--set(0x10) disable
  OLED_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
  OLED_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7) 
  OLED_WrCmd(0xaf);//--turn on oled panel
  OLED_Fill(0x00); //初始清屏
  OLED_Set_Pos(0,0);
} 

/****************SPI***************************/
/*------------写一个数据：片选拉高-----------*/
void LCD_WrDat(unsigned char dat)
{
  unsigned char i=8;
  LCD_DCout(Bit_SET);
  for(i=0;i<8;i++)				//传送8位数据：时钟线拉低有效
  {
    LCD_SCLout(Bit_RESET);
    if(dat&0x80)	LCD_SDAout(Bit_SET);
    else	LCD_SDAout(Bit_RESET);
    LCD_SCLout(Bit_SET);
    dat<<=1;
  }
}
/*------------写命令：片选拉低-------------*/
void LCD_WrCmd(unsigned char cmd)
{
  unsigned char i=8;
  LCD_DCout(Bit_RESET);
  for(i=0;i<8;i++)			//传送8位数据：时钟线拉低有效
  {
    LCD_SCLout(Bit_RESET);
    if(cmd&0x80)	LCD_SDAout(Bit_SET);
    else LCD_SDAout(Bit_RESET);
    LCD_SCLout(Bit_SET);
    cmd<<=1;
  }
  LCD_DCout(Bit_SET);
}
/*----------------设置坐标------------------*/
void LCD_Set_Pos(unsigned char x, unsigned char y)
{
  LCD_WrCmd(0xb0+y);
  LCD_WrCmd(((x&0xf0)>>4)|0x10);
  LCD_WrCmd((x&0x0f)|0x01);
}
/*----------------全屏显示-----------------*/
void LCD_Fill(unsigned char bmp_dat)
{
  unsigned char y,x;
  for(y=0;y<8;y++)
  {
    LCD_WrCmd(0xb0+y);
    LCD_WrCmd(0x01);
    LCD_WrCmd(0x10);
    for(x=0;x<X_WIDTH;x++)
    {
      LCD_WrDat(bmp_dat);
    }
  }
}
/*---------------LCD复位-----------------*/
void LCD_CLS(void)
{
  unsigned char y,x;
#ifdef OLED_WORK_MODE_I2C
  for(y=0;y<8;y++)
  {
    OLED_WrCmd(0xb0+y);
    OLED_WrCmd(0x01);
    OLED_WrCmd(0x10);
    for(x=0;x<X_WIDTH;x++)
      OLED_WrDat(0); 
  }
#else  
  for(y=0;y<8;y++)
  {
    LCD_WrCmd(0xb0+y);
    LCD_WrCmd(0x01);
    LCD_WrCmd(0x10);
    for(x=0;x<X_WIDTH;x++)
      LCD_WrDat(0); 
  }
#endif
  
}
/*------显示6X8一组标准的ASCII字符串，显示坐标为（x，y）------*/
void LCD_P6x8Str(unsigned char x,unsigned char  y,unsigned char ch[])
{
  unsigned char c=0,i=0,j=0;
  while (ch[j]!='\0')
  {
    c =ch[j]-32;
#ifdef OLED_WORK_MODE_I2C
    if(x>126){x=0;y++;}
    OLED_Set_Pos(x,y);
    for(i=0;i<6;i++)
      OLED_WrDat(F6x8[c][i]);
#else    
    if(x>126){x=0;y++;}
    LCD_Set_Pos(x,y);
    for(i=0;i<6;i++)
      LCD_WrDat(F6x8[c][i]);
#endif
    x+=6;
    j++;
  }
}
//显示一个6X8的字符
void LCD_P6x8Char(unsigned char x,unsigned char  y,unsigned char ucData)
{
  unsigned char i, ucDataTmp;
  ucDataTmp = ucData-32;
  if(x > 126)
  {
    x= 0;
    y++;
  }
#ifdef OLED_WORK_MODE_I2C
  OLED_Set_Pos(x, y);
  for(i = 0; i < 6; i++)
  {
    OLED_WrDat(F6x8[ucDataTmp][i]);
  }
#else
  LCD_Set_Pos(x, y);
  for(i = 0; i < 6; i++)
  {
    LCD_WrDat(F6x8[ucDataTmp][i]);
  }
#endif
}
/*--------------显示6X8的浮点数--------------*/
void write_6_8_number(unsigned char x,unsigned char y, float number)
{
  unsigned char i=0;
  unsigned char temp[16];
  unsigned char *point=temp;
  float decimal;
  int data;
  if(number<0)
  {
    temp[0]='-';
    LCD_P6x8Char(x,y,temp[0]);
    x+=6;
    number=-number;
  }
  data=(int)number;
  decimal=number-data;					//小数部分
  
  if(data>=1000000000)           //是否能被10^9整除
  {
    temp[i]=48+data/1000000000;
    data=data%1000000000;
    i++;
  }
  if(data>=100000000)           //是否能被10^8整除
  {
    temp[i]=48+data/100000000;
    data=data%100000000;
    i++;
  }
  else
    if(data<100000000&&i!=0)
    {
      temp[i]=0+48;
      i++;
    }
  if(data>=10000000)           //是否能被10^7整除
  {
    temp[i]=48+data/10000000;
    data=data%10000000;
    i++;
  }
  else
    if(data<10000000&&i!=0)
    {
      temp[i]=0+48;
      i++;
    }
  if(data>=1000000)           //是否能被10^6整除
  {
    temp[i]=48+data/1000000;
    data=data%1000000;
    i++;
  }
  else
    if(data<1000000&&i!=0)
    {
      temp[i]=0+48;
      i++;
    }
  if(data>=100000)           //是否能被100000整除
  {
    temp[i]=48+data/100000;
    data=data%100000;
    i++;
  }
  else
    if(data<100000&&i!=0)
    {
      temp[i]=0+48;
      i++;
    }
  if(data>=10000)           //是否能被10000整除
  {
    temp[i]=48+data/10000;
    data=data%10000;
    i++;
  }
  else
    if(data<10000&&i!=0)
    {
      temp[i]=0+48;
      i++;
    }
  if(data>=1000)           //是否能被1000整除
  {
    temp[i]=48+data/1000;
    data=data%1000;
    i++;
  }
  else
    if(data<1000&&i!=0)
    {
      temp[i]=0+48;
      i++;
    }
  if(data>=100)              //是否能被100整除
  {
    temp[i]=48+data/100;
    data=data%100;
    i++;
  }
  else
    if(data<100&&i!=0)
    {
      temp[i]=0+48;
      i++;
    }
  if(data>=10)                  //是否能被10整除
  {
    temp[i]=48+data/10;
    data=data%10;
    i++;
  }
  else
    if(data<10&&i!=0)
    {
      temp[i]=48;
      i++;
    }
  temp[i]=48+data;
  if(decimal>=0.0001f)           //判断是否有小数部分
  {
    i++;
    temp[i]='.';                //显示小数点
    i++;
    data=(int)(decimal*1000);
    temp[i]=48+data/100;
    data=data%100;
    i++;
    if(data>0)
    {
      temp[i]=48+data/10;
      data=data%10;
    }
    if(data>=0)
    {
      i++;
      temp[i]=data+48;
    }
  }
  i++;
  temp[i]='\0';
  LCD_P6x8Str(x,y,point);
}
/*------显示8X16一组标准的ASCII字符串，显示坐标为（x，y）------*/
void LCD_P8x16Str(unsigned char x,unsigned char  y,unsigned char ch[])
{
  unsigned char c=0,i=0,j=0;
  while (ch[j]!='\0')
  {
    c =ch[j]-32;
    if(x>120){x=0;y++;}
#ifdef OLED_WORK_MODE_I2C   
    OLED_Set_Pos(x,y);
    for(i=0;i<8;i++)
      OLED_WrDat(F8X16[c*16+i]);
    OLED_Set_Pos(x,y+1);
    for(i=0;i<8;i++)
      OLED_WrDat(F8X16[c*16+i+8]);
#else
    LCD_Set_Pos(x,y);
    for(i=0;i<8;i++)
      LCD_WrDat(F8X16[c*16+i]);
    LCD_Set_Pos(x,y+1);
    for(i=0;i<8;i++)
      LCD_WrDat(F8X16[c*16+i+8]);    
#endif    
    x+=8;
    j++;
  }
}
//显示一个8X16的字符
void LCD_P8x16Char(unsigned char x,unsigned char  y,unsigned char ch)
{
  unsigned char c=0,i=0,j=0;
  c =ch-32;
  if(x>120){x=0;y++;}
#ifdef OLED_WORK_MODE_I2C    
  OLED_Set_Pos(x,y);
  for(i=0;i<8;i++)
    OLED_WrDat(F8X16[c*16+i]);
  OLED_Set_Pos(x,y+1);
  for(i=0;i<8;i++)
    OLED_WrDat(F8X16[c*16+i+8]);
#else
  LCD_Set_Pos(x,y);
  for(i=0;i<8;i++)
    LCD_WrDat(F8X16[c*16+i]);
  LCD_Set_Pos(x,y+1);
  for(i=0;i<8;i++)
    LCD_WrDat(F8X16[c*16+i+8]);  
#endif   
  x+=8;
  j++;
}
/*---------------------显示8X16的浮点数--------------------*/
void write_8_16_number(unsigned char x,unsigned char y, float number)
{
  unsigned char i=0;
  unsigned char temp[16];
  unsigned char *point=temp;
  float decimal;
  int data;
  
  if(number<0)
  {
    temp[0]='-';
    LCD_P8x16Char(x,y,temp[0]);
    x+=1;
    number=-number;
  }
  data=(int)number;
  decimal=number-data;     //小数部分
  if(data>=1000)           //是否可被1000整除
  {
    temp[i]=48+data/1000;
    data=data%1000;
    i++;
  }
  if(data>=100)              //可否被100整除
  {
    temp[i]=48+data/100;
    data=data%100;
    i++;
  }
  else
    if(data<100&&i!=0)
    {
      temp[i]=0+48;
      i++;
    }
  if(data>=10)                  //可否被10整除
  {
    temp[i]=48+data/10;
    data=data%10;
    i++;
  }
  else
    if(data<10&&i!=0)
    {
      temp[i]=48;
      i++;
    }
  temp[i]=48+data;
  if(decimal>=0.0001f)           //判断是够为小数
  {
    i++;
    temp[i]='.';                //显示小数点
    i++;
    data=(int)(decimal*1000);
    temp[i]=48+data/100;
    data=data%100;
    i++;
    if(data>0)
    {
      temp[i]=48+data/10;
      data=data%10;
    }
    if(data>=0)
    {
      i++;
      temp[i]=data+48;
    }
  }
  i++;
  temp[i]='\0';
  LCD_P8x16Str(x,y,point);
}

//--------------------------------------------------------------
// Prototype      : void write_16_16_CN(unsigned char x, unsigned char y, unsigned char N)
// Calls          : 
// Parameters     : x,y -- 起始点坐标(x:0~127, y:0~7); N:汉字在codetab.h中的索引
// Description    : 显示codetab.h中的汉字,16*16点阵
//--------------------------------------------------------------
void write_16_16_CN(unsigned char x, unsigned char y, unsigned char N)
{
  unsigned char wm=0;
  unsigned int  adder=32*N;
#ifdef OLED_WORK_MODE_I2C    
  OLED_Set_Pos(x , y);
  for(wm = 0;wm < 16;wm++)
  {
    OLED_WrDat(F16x16[adder]);
    adder += 1;
  }
  OLED_Set_Pos(x,y + 1);
  for(wm = 0;wm < 16;wm++)
  {
    OLED_WrDat(F16x16[adder]);
    adder += 1;
  }
#else
  LCD_Set_Pos(x , y);
  for(wm = 0;wm < 16;wm++)
  {
    LCD_WrDat(F16x16[adder]);
    adder += 1;
  }
  LCD_Set_Pos(x,y + 1);
  for(wm = 0;wm < 16;wm++)
  {
    LCD_WrDat(F16x16[adder]);
    adder += 1;
  }
#endif  		
}

void LCD_clear_L(unsigned char x,unsigned char y)
{
#ifdef OLED_WORK_MODE_I2C  
  OLED_WrCmd(0xb0+y);
  OLED_WrCmd(0x01);
  OLED_WrCmd(0x10);
  OLED_Set_Pos(x,y);
  for(;x<X_WIDTH;x++)
  {
    OLED_WrDat(0);
  }
#else
  LCD_WrCmd(0xb0+y);
  LCD_WrCmd(0x01);
  LCD_WrCmd(0x10);
  LCD_Set_Pos(x,y);
  for(;x<X_WIDTH;x++)
  {
    LCD_WrDat(0);
  }        
#endif
}

void Draw_Logo(void)
{
  unsigned int ii=0;
  unsigned char x,y;
#ifdef OLED_WORK_MODE_I2C    
  for(y=0;y<8;y++)
  {
    OLED_Set_Pos(0,y);
    for(x=0;x<128;x++)
    {
      OLED_WrDat(LOGO128x64[ii++]);
    }
  }
#else
  for(y=0;y<8;y++)
  {
    LCD_Set_Pos(0,y);
    for(x=0;x<128;x++)
    {
      LCD_WrDat(LOGO128x64[ii++]);
    }
  }
#endif    
}

void Draw_Logo1(void)
{
  unsigned int ii=0;
  unsigned char x,y;
#ifdef OLED_WORK_MODE_I2C    
  for(y=0;y<8;y++)
  {
    OLED_Set_Pos(0,y);
    for(x=0;x<128;x++)
    {
      OLED_WrDat(YuYi_TC[ii++]);
      Delay_Ms(1);
    }
  }
#else
  for(y=0;y<8;y++)
  {
    LCD_Set_Pos(0,y);
    for(x=0;x<128;x++)
    {
      LCD_WrDat(YuYi_TC[ii++]);
      Delay_Ms(2);
    }
  }
#endif
  Delay_Ms(500);
  LCD_CLS();
}


//显示屏初始化
void OLEDInit(void)
{
  LCD_RSTout(Bit_RESET);
  Delay_Ms(100);
  LCD_RSTout(Bit_SET);//等待RC复位完毕
  
  LCD_WrCmd(0xae);		//--turn off oled panel
  LCD_WrCmd(0x00);		//---set low column address
  LCD_WrCmd(0x10);		//---set high column address
  LCD_WrCmd(0x40);		//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
  LCD_WrCmd(0x81);		//--set contrast control register
  LCD_WrCmd(0xcf); 		// Set SEG Output Current Brightness
  LCD_WrCmd(0xa0);		//--Set SEG/Column Mapping     0xa0???? 0xa1??
  LCD_WrCmd(0xc0);		//Set COM/Row Scan Direction   0xc0???? 0xc8??
  LCD_WrCmd(0xa6);		//--set normal display
  LCD_WrCmd(0xa8);		//--set multiplex ratio(1 to 64)
  LCD_WrCmd(0x3f);		//--1/64 duty
  LCD_WrCmd(0xd3);		//-set display offset Shift Mapping RAM Counter (0x00~0x3F)
  LCD_WrCmd(0x00);		//-not offset
  LCD_WrCmd(0xd5);		//--set display clock divide ratio/oscillator frequency
  LCD_WrCmd(0x80);		//--set divide ratio, Set Clock as 100 Frames/Sec
  LCD_WrCmd(0xd9);		//--set pre-charge period
  LCD_WrCmd(0xf1);		//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  LCD_WrCmd(0xda);		//--set com pins hardware configuration
  LCD_WrCmd(0x12);
  LCD_WrCmd(0xdb);		//--set vcomh
  LCD_WrCmd(0x40);		//Set VCOM Deselect Level
  LCD_WrCmd(0x20);		//-Set Page Addressing Mode (0x00/0x01/0x02)
  LCD_WrCmd(0x02);		//
  LCD_WrCmd(0x8d);		//--set Charge Pump enable/disable
  LCD_WrCmd(0x14);		//--set(0x10) disable
  LCD_WrCmd(0xa4);		// Disable Entire Display On (0xa4/0xa5)
  LCD_WrCmd(0xa6);		// Disable Inverse Display On (0xa6/a7)
  LCD_WrCmd(0xaf);		//--turn on oled panel
  LCD_Fill(0x00);  		//初始清屏
  LCD_Set_Pos(0,0);
}

/***************************************************
函数名: void OLED_Init(void)
说明:	OLED显示屏初始化
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void OLED_Init(void)
{
#ifdef OLED_WORK_MODE_I2C 
  OLED_GPIO_Init();
  //OLED_Init_I2C();
  ssd1306_begin(SSD1306_SWITCHCAPVCC);   
#else
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIO_OLED);// Enable the GPIO port that is used for the on-board LED.
  GPIOPinTypeGPIOOutput(GPIO_OLED, GPIO_PIN_2);
  GPIOPinTypeGPIOOutput(GPIO_OLED, GPIO_PIN_3);
  GPIOPinTypeGPIOOutput(GPIO_OLED, GPIO_PIN_4);
  GPIOPinTypeGPIOOutput(GPIO_OLED, GPIO_PIN_5);	
  Delay_Ms(2);
  //OLEDInit();
  LCD_RSTout(Bit_RESET);
  Delay_Ms(100);
  LCD_RSTout(Bit_SET);//等待RC复位完毕
  ssd1306_begin(SSD1306_SWITCHCAPVCC);  
#endif
//  Draw_Logo1();
  LCD_CLS();
}


void OLED_Init_Fast(void)
{
#ifdef OLED_WORK_MODE_I2C 
  OLED_GPIO_Init();
  //OLED_Init_I2C();
  ssd1306_begin(SSD1306_SWITCHCAPVCC);  
#else
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIO_OLED);// Enable the GPIO port that is used for the on-board LED.
  GPIOPinTypeGPIOOutput(GPIO_OLED, GPIO_PIN_2);
  GPIOPinTypeGPIOOutput(GPIO_OLED, GPIO_PIN_3);
  GPIOPinTypeGPIOOutput(GPIO_OLED, GPIO_PIN_4);
  GPIOPinTypeGPIOOutput(GPIO_OLED, GPIO_PIN_5);	
  Delay_Ms(10);
  //OLEDInit(); 
  LCD_RSTout(Bit_RESET);
  Delay_Ms(100);
  LCD_RSTout(Bit_SET);//等待RC复位完毕
  ssd1306_begin(SSD1306_SWITCHCAPVCC);  	
#endif
}


#include "Headfile.h"
#include "myiic.h"
#include "WP_DataType.h"
#include "SPL06.h"


#define PRESSURE_REG 0X00
#define TEMP_REG 0X03
#define PRS_CFG 0x06
#define TMP_CFG 0x07
#define MEAS_CFG 0x08
#define SPL06_REST_VALUE 0x09
#define SPL06_REST_REG 0x0C
#define PRODUCT_ID 0X0D
//#define HW_ADR 0x77 //SDO HIGH OR NC
#define HW_ADR (0x76) //SDO LOW
#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1


struct spl0601_calib_param_t {
  int16 c0;
  int16 c1;
  int32 c00;
  int32 c10;
  int16 c01;
  int16 c11;
  int16 c20;
  int16 c21;
  int16 c30;
};

struct spl0601_t {
  struct spl0601_calib_param_t calib_param;/**<calibration data*/
  uint8 chip_id; /**<chip id*/
  int32 i32rawPressure;
  int32 i32rawTemperature;
  int32 i32kP;
  int32 i32kT;
};


struct spl0601_t spl0601;
struct spl0601_t *p_spl0601;
void spl0601_write(uint8 hwadr, uint8 regadr, uint8 val);
uint8 spl0601_read(uint8 hwadr, uint8 regadr);
void spl0601_get_calib_param(void);


/*****************************************************************************
函 数 名  : spl0601_write
功能描述  : I2C 寄存器写入子函数
输入参数  : uint8 hwadr   硬件地址
uint8 regadr  寄存器地址
uint8 val     值
输出参数  : 无
返 回 值  :
调用函数  :
被调函数  :

修改历史      :
1.日    期   : 2015年11月30日
作    者   : WL
修改内容   : 新生成函数

*****************************************************************************/
void spl0601_write(unsigned char hwadr, unsigned char regadr, unsigned char val)
{
  Single_WriteI2C(hwadr,regadr,val);
}

/*****************************************************************************
函 数 名  : spl0601_read
功能描述  : I2C 寄存器读取子函数
输入参数  : uint8 hwadr   硬件地址
uint8 regadr  寄存器地址
输出参数  :
返 回 值  : uint8 读出值
调用函数  :
被调函数  :

修改历史      :
1.日    期   : 2015年11月30日
作    者   : WL
修改内容   : 新生成函数

*****************************************************************************/
uint8 spl0601_read(unsigned char hwadr, unsigned char regadr)
{
  uint8 reg_data;
  reg_data=Single_ReadI2C(hwadr,regadr);
  return reg_data;
}

/*****************************************************************************
函 数 名  : spl0601_init
功能描述  : SPL06-01 初始化函数
输入参数  : void
输出参数  : 无
返 回 值  :
调用函数  :
被调函数  :

修改历史      :
1.日    期   : 2015年11月30日
作    者   : WL
修改内容   : 新生成函数

*****************************************************************************/
void spl0601_init(void)
{
  p_spl0601 = &spl0601; /* read Chip Id */
  p_spl0601->i32rawPressure = 0;
  p_spl0601->i32rawTemperature = 0;
  p_spl0601->chip_id = 0x34;
  spl0601_get_calib_param();
  spl0601_rateset(PRESSURE_SENSOR,64, 32);
  spl0601_rateset(TEMPERATURE_SENSOR,128, 2);
  //Start background measurement
  spl0601_start_continuous(CONTINUOUS_P_AND_T);
}


/*****************************************************************************
函 数 名  : spl0601_rateset
功能描述  :  设置温度传感器的每秒采样次数以及过采样率
输入参数  : uint8 u8OverSmpl  过采样率         Maximal = 128
uint8 u8SmplRate  每秒采样次数(Hz) Maximal = 128
uint8 iSensor     0: Pressure; 1: Temperature
输出参数  : 无
返 回 值  : 无
调用函数  :
被调函数  :

修改历史      :
1.日    期   : 2015年11月24日
作    者   : WL
修改内容   : 新生成函数

*****************************************************************************/
void spl0601_rateset(uint8 iSensor, uint8 u8SmplRate, uint8 u8OverSmpl)
{
  uint8 reg = 0;
  int32 i32kPkT = 0;
  switch(u8SmplRate)
  {
  case 2:
    reg |= (1<<4);//左移动4位（原5位），原厂提供例程有问题，20180410无名小哥改
    break;
  case 4:
    reg |= (2<<4);
    break;
  case 8:
    reg |= (3<<4);
    break;
  case 16:
    reg |= (4<<4);
    break;
  case 32:
    reg |= (5<<4);
    break;
  case 64:
    reg |= (6<<4);
    break;
  case 128:
    reg |= (7<<4);
    break;
  case 1:
  default:
    break;
  }
  switch(u8OverSmpl)
  {
  case 2:
    reg |= 1;
    i32kPkT = 1572864;
    break;
  case 4:
    reg |= 2;
    i32kPkT = 3670016;
    break;
  case 8:
    reg |= 3;
    i32kPkT = 7864320;
    break;
  case 16:
    i32kPkT = 253952;
    reg |= 4;
    break;
  case 32:
    i32kPkT = 516096;
    reg |= 5;
    break;
  case 64:
    i32kPkT = 1040384;
    reg |= 6;
    break;
  case 128:
    i32kPkT = 2088960;
    reg |= 7;
    break;
  case 1:
  default:
    i32kPkT = 524288;
    break;
  }
  
  if(iSensor == 0)
  {
    p_spl0601->i32kP = i32kPkT;
    spl0601_write(HW_ADR, 0x06, reg);
    if(u8OverSmpl > 8)
    {
      reg = spl0601_read(HW_ADR, 0x09);
      spl0601_write(HW_ADR, 0x09, reg | 0x04);
    }
  }
  if(iSensor == 1)
  {
    p_spl0601->i32kT = i32kPkT;
    spl0601_write(HW_ADR, 0x07, reg|0x80);  //Using mems temperature
    if(u8OverSmpl > 8)
    {
      reg = spl0601_read(HW_ADR, 0x09);
      spl0601_write(HW_ADR, 0x09, reg | 0x08);
    }
  }
}

/*****************************************************************************
函 数 名  : spl0601_get_calib_param
功能描述  : 获取校准参数
输入参数  : void
输出参数  : 无
返 回 值  :
调用函数  :
被调函数  :

修改历史      :
1.日    期   : 2015年11月30日
作    者   : WL
修改内容   : 新生成函数

*****************************************************************************/
void spl0601_get_calib_param(void)
{
  uint32 h;
  uint32 m;
  uint32 l;
  h =  spl0601_read(HW_ADR, 0x10);
  l  =  spl0601_read(HW_ADR, 0x11);
  p_spl0601->calib_param.c0 = (int16)h<<4 | l>>4;
  p_spl0601->calib_param.c0 = (p_spl0601->calib_param.c0&0x0800)?(0xF000|p_spl0601->calib_param.c0):p_spl0601->calib_param.c0;
  h =  spl0601_read(HW_ADR, 0x11);
  l  =  spl0601_read(HW_ADR, 0x12);
  p_spl0601->calib_param.c1 = (int16)(h&0x0F)<<8 | l;
  p_spl0601->calib_param.c1 = (p_spl0601->calib_param.c1&0x0800)?(0xF000|p_spl0601->calib_param.c1):p_spl0601->calib_param.c1;
  h =  spl0601_read(HW_ADR, 0x13);
  m =  spl0601_read(HW_ADR, 0x14);
  l =  spl0601_read(HW_ADR, 0x15);
  p_spl0601->calib_param.c00 = (int32)h<<12 | (int32)m<<4 | (int32)l>>4;
  p_spl0601->calib_param.c00 = (p_spl0601->calib_param.c00&0x080000)?(0xFFF00000|p_spl0601->calib_param.c00):p_spl0601->calib_param.c00;
  h =  spl0601_read(HW_ADR, 0x15);
  m =  spl0601_read(HW_ADR, 0x16);
  l =  spl0601_read(HW_ADR, 0x17);
  p_spl0601->calib_param.c10 = (int32)h<<16 | (int32)m<<8 | l;
  p_spl0601->calib_param.c10 = (p_spl0601->calib_param.c10&0x080000)?(0xFFF00000|p_spl0601->calib_param.c10):p_spl0601->calib_param.c10;
  h =  spl0601_read(HW_ADR, 0x18);
  l  =  spl0601_read(HW_ADR, 0x19);
  p_spl0601->calib_param.c01 = (int16)h<<8 | l;
  h =  spl0601_read(HW_ADR, 0x1A);
  l  =  spl0601_read(HW_ADR, 0x1B);
  p_spl0601->calib_param.c11 = (int16)h<<8 | l;
  h =  spl0601_read(HW_ADR, 0x1C);
  l  =  spl0601_read(HW_ADR, 0x1D);
  p_spl0601->calib_param.c20 = (int16)h<<8 | l;
  h =  spl0601_read(HW_ADR, 0x1E);
  l  =  spl0601_read(HW_ADR, 0x1F);
  p_spl0601->calib_param.c21 = (int16)h<<8 | l;
  h =  spl0601_read(HW_ADR, 0x20);
  l  =  spl0601_read(HW_ADR, 0x21);
  p_spl0601->calib_param.c30 = (int16)h<<8 | l;
}


/*****************************************************************************
函 数 名  : spl0601_start_temperature
功能描述  : 发起一次温度测量
输入参数  : void
输出参数  : 无
返 回 值  :
调用函数  :
被调函数  :

修改历史      :
1.日    期   : 2015年11月30日
作    者   : WL
修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_temperature(void)
{
  spl0601_write(HW_ADR, 0x08, 0x02);
}

/*****************************************************************************
函 数 名  : spl0601_start_pressure
功能描述  : 发起一次压力值测量
输入参数  : void
输出参数  : 无
返 回 值  :
调用函数  :
被调函数  :

修改历史      :
1.日    期   : 2015年11月30日
作    者   : WL
修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_pressure(void)
{
  spl0601_write(HW_ADR, 0x08, 0x01);
}

/*****************************************************************************
函 数 名  : spl0601_start_continuous
功能描述  : Select node for the continuously measurement
输入参数  : uint8 mode  1: pressure; 2: temperature; 3: pressure and temperature
输出参数  : 无
返 回 值  :
调用函数  :
被调函数  :

修改历史      :
1.日    期   : 2015年11月25日
作    者   : WL
修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_continuous(uint8 mode)
{
  spl0601_write(HW_ADR, 0x08, mode+4);
}


/*****************************************************************************
函 数 名  : spl0601_get_raw_temp
功能描述  : 获取温度的原始值，并转换成32Bits整数
输入参数  : void
输出参数  : 无
返 回 值  :
调用函数  :
被调函数  :

修改历史      :
1.日    期   : 2015年11月30日
作    者   : WL
修改内容   : 新生成函数

*****************************************************************************/
void spl0601_get_raw_temp(void)
{
  uint8 h[3] = {0};
  h[0] = spl0601_read(HW_ADR, 0x03);
  h[1] = spl0601_read(HW_ADR, 0x04);
  h[2] = spl0601_read(HW_ADR, 0x05);
  p_spl0601->i32rawTemperature = (int32)h[0]<<16 | (int32)h[1]<<8 | (int32)h[2];
  p_spl0601->i32rawTemperature= (p_spl0601->i32rawTemperature&0x800000) ? (0xFF000000|p_spl0601->i32rawTemperature) : p_spl0601->i32rawTemperature;
}

/*****************************************************************************
函 数 名  : spl0601_get_raw_pressure
功能描述  : 获取压力原始值，并转换成32bits整数
输入参数  : void
输出参数  : 无
返 回 值  :
调用函数  :
被调函数  :

修改历史      :
1.日    期   : 2015年11月30日
作    者   : WL
修改内容   : 新生成函数

*****************************************************************************/
void spl0601_get_raw_pressure(void)
{
  uint8 h[3];
  h[0] = spl0601_read(HW_ADR, 0x00);
  h[1] = spl0601_read(HW_ADR, 0x01);
  h[2] = spl0601_read(HW_ADR, 0x02);
  p_spl0601->i32rawPressure = (int32)h[0]<<16 | (int32)h[1]<<8 | (int32)h[2];
  p_spl0601->i32rawPressure= (p_spl0601->i32rawPressure&0x800000) ? (0xFF000000|p_spl0601->i32rawPressure) : p_spl0601->i32rawPressure;
}


/*****************************************************************************
函 数 名  : spl0601_get_temperature
功能描述  : 在获取原始值的基础上，返回浮点校准后的温度值
输入参数  : void
输出参数  : 无
返 回 值  :
调用函数  :
被调函数  :

修改历史      :
1.日    期   : 2015年11月30日
作    者   : WL
修改内容   : 新生成函数

*****************************************************************************/
float spl0601_get_temperature(void)
{
  float fTCompensate;
  float fTsc;
  
  fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
  fTCompensate =  p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
  return fTCompensate;
}

/*****************************************************************************
函 数 名  : spl0601_get_pressure
功能描述  : 在获取原始值的基础上，返回浮点校准后的压力值
输入参数  : void
输出参数  : 无
返 回 值  :
调用函数  :
被调函数  :

修改历史      :
1.日    期   : 2015年11月30日
作    者   : WL
修改内容   : 新生成函数

*****************************************************************************/
float spl0601_get_pressure(void)
{
  float fTsc, fPsc;
  float qua2, qua3;
  float fPCompensate;
  
  fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
  fPsc = p_spl0601->i32rawPressure / (float)p_spl0601->i32kP;
  qua2 = p_spl0601->calib_param.c10 + fPsc * (p_spl0601->calib_param.c20 + fPsc* p_spl0601->calib_param.c30);
  qua3 = fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);
  //qua3 = 0.9f *fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);
  
  fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
  //fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + 0.9f *fTsc  * p_spl0601->calib_param.c01 + qua3;
  return fPCompensate;
}


float temperature;
float pressure;
uint8_t baro_flag=0;
float user_spl0601_get()//气压计数据获取状态机
{
  static uint16_t spl06_cnt=0;
  spl06_cnt++;
  if(spl06_cnt==1)//1
  {
    spl0601_get_raw_temp();
    temperature = spl0601_get_temperature();
  }
  else  if(spl06_cnt==22)//105ms
  {
    spl0601_get_raw_pressure();
    pressure = spl0601_get_pressure();
    spl06_cnt=0;
    baro_flag=1;
  }
  return 0;
}
void SPL06_Init(void)//气压计初始化配置
{
  spl0601_init();
}

void SPL06_Read_Data(float *baro_t,float *baro_p)//气压计数据采集
{
  user_spl0601_get();
  *baro_p=pressure;
  *baro_t=temperature;
}


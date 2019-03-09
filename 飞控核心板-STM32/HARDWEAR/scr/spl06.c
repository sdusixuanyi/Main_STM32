#include "spl06.h"

struct calib_param SPL06_calparam;    //气压计校准值
struct SPL06_DATA SPL06_data;

int HEIGHT_process_complete = 1;    //1时高度计算完成

extern struct Attitude_float attitude;

void SPL06_init()
{
	unsigned char status;
	//while(!SPL06_check());
	
	do i2c_read(SPL06_ADDR, MEAS_CFG_REG, &status, 1);
	while((status&MEAS_CFG_COEF_RDY) != MEAS_CFG_COEF_RDY);
	
	SPL06_get_calib_param(1);
	
	do i2c_read(SPL06_ADDR, MEAS_CFG_REG, &status, 1);
	while((status&MEAS_CFG_SENSOR_RDY) != MEAS_CFG_SENSOR_RDY);
	
	SPL06_set_rate(PRESSURE_SENSOR, 128, 32);   //设置采样率和过采样率
	SPL06_set_rate(TEMPERATURE_SENSOR, 32, 8);
	
	i2c_byte_write(SPL06_ADDR, MEAS_CFG_REG, MEAS_CFG_MEAS_CTR_BACKGROUND_PSR_TMP);     //设置采集模式
	
	delay_ms(3000);
	
	SPL06_info_update();
	SPL06_info_update();
	SPL06_data.fGround_Alt = SPL06_data.fALT;    //设定地面的海拔高度
}

void SPL06_get_raw_data()
{
	//获取温度
  uint8_t u8Data[3] = {0};
    
//	i2c_read(SPL06_ADDR, 0x03, &(u8Data[0]), 1);
//	i2c_read(SPL06_ADDR, 0x04, &(u8Data[1]), 1);
//	i2c_read(SPL06_ADDR, 0x05, &(u8Data[2]), 1);
	
	i2c_read(SPL06_ADDR, 0x03, u8Data, 3);

	SPL06_data.i32RawTemperature = (int32_t)u8Data[0] << 16 | \
																				(int32_t)u8Data[1] << 8  | \
																				(int32_t)u8Data[2];
	
	SPL06_data.i32RawTemperature = (SPL06_data.i32RawTemperature & 0x800000)   ? \
																				(0xFF000000 | SPL06_data.i32RawTemperature) : \
																				(SPL06_data.i32RawTemperature);              //存数据
	
	//获取气压
//	i2c_read(SPL06_ADDR, 0x00, &(u8Data[0]), 1);
//	i2c_read(SPL06_ADDR, 0x01, &(u8Data[1]), 1);
//	i2c_read(SPL06_ADDR, 0x02, &(u8Data[2]), 1);
	i2c_read(SPL06_ADDR, 0x00, u8Data, 3);
	
	SPL06_data.i32RawPressure = (int32_t)u8Data[0] << 16 | \
																		 (int32_t)u8Data[1] << 8  | \
																		 (int32_t)u8Data[2];
	SPL06_data.i32RawPressure = (SPL06_data.i32RawPressure & 0x800000)   ? \
																		 (0xFF000000 | SPL06_data.i32RawPressure) : \
																		 (SPL06_data.i32RawPressure);                //存数据
}

int SPL06_check()
{
	unsigned char ID = 0;
	i2c_read(SPL06_ADDR, ID_REG, &ID, 1);
	
	if(ID == PRODUCT_ID)  return 1;
	else                  return 0;
}

void SPL06_set_rate(unsigned char u8_Sensor, unsigned char u8_SmplRate, unsigned char u8_OverSmpl)
{
	unsigned char u8Reg = 0;
	int i32KPkT = 0;
	
	switch(u8_SmplRate)
	{
			case 2:
					u8Reg |= (1 << 5);
					break;
			case 4:
					u8Reg |= (2 << 5);
					break;
			case 8:
					u8Reg |= (3 << 5);
					break;
			case 16:
					u8Reg |= (4 << 5);
					break;
			case 32:
					u8Reg |= (5 << 5);
					break;
			case 64:
					u8Reg |= (6 << 5);
					break;
			case 128:
					u8Reg |= (7 << 5);
					break;
			case 1:
			default:
					break;
	}
	
	 switch(u8_OverSmpl)
	 {
			case 2:
					u8Reg |= 1;
					i32KPkT = 1572864;
					break;
			case 4:
					u8Reg |= 2;
					i32KPkT = 3670016;
					break;
			case 8:
					u8Reg |= 3;
					i32KPkT = 7864320;
					break;
			case 16:
					i32KPkT = 253952;
					u8Reg |= 4;
					break;
			case 32:
					i32KPkT = 516096;
					u8Reg |= 5;
					break;
			case 64:
					i32KPkT = 1040384;
					u8Reg |= 6;
					break;
			case 128:
					i32KPkT = 2088960;
					u8Reg |= 7;
					break;
			case 1:
			default:
					i32KPkT = 524288;
					break;
	 }
	 
	 if(u8_Sensor == 0)
    {
        SPL06_data.i32KP = i32KPkT;
        i2c_byte_write(SPL06_ADDR, PRS_CFG_REG, u8Reg);
        if(u8_OverSmpl > 8)
        {
            i2c_read(SPL06_ADDR, CFG_REG, &u8Reg, 1);
            i2c_byte_write(SPL06_ADDR, CFG_REG, u8Reg | 0x04);
        }
    }
    
    if(u8_Sensor == 1)
    {
        SPL06_data.i32KT = i32KPkT;
        
        //Using mems temperature
        i2c_byte_write(SPL06_ADDR, TMP_CFG_REG, u8Reg|0x80);  
        
        if(u8_OverSmpl > 8)
        {
            i2c_read(SPL06_ADDR, CFG_REG, &u8Reg, 1);
            i2c_byte_write(SPL06_ADDR, CFG_REG, u8Reg | 0x08);
        }
    }
}


void SPL06_get_calib_param(int mode)
{
    unsigned long h;
    unsigned long m;
    unsigned long l;
	  
	  unsigned char temp_h, temp_m, temp_l;
	
    if(mode == 0)
		{
        SPL06_calparam.c0 = 204;          //人工设定校准值  数据来源中科浩电
				SPL06_calparam.c1 = -261;
				SPL06_calparam.c00 = 80469;
				SPL06_calparam.c10 = -54769;
				SPL06_calparam.c01 = -2803;
				SPL06_calparam.c11 = 1226;
				SPL06_calparam.c20 = -10787;
				SPL06_calparam.c21 = 183;
				SPL06_calparam.c30 = -1603;
		}
    if(mode == 1)
		{			
				i2c_read(SPL06_ADDR, 0x10, &temp_h, 1);
				i2c_read(SPL06_ADDR, 0x11, &temp_l, 1);
			  h = temp_h;
			  l = temp_l;
				SPL06_calparam.c0 = (short)h<<4 | l>>4;
				SPL06_calparam.c0 = (SPL06_calparam.c0&0x0800)?(0xF000|SPL06_calparam.c0):SPL06_calparam.c0;
				
				i2c_read(SPL06_ADDR, 0x11,&temp_h, 1);
				i2c_read(SPL06_ADDR, 0x12, &temp_l, 1);
			  h = temp_h;
			  l = temp_l;
				SPL06_calparam.c1 = (short)(h&0x0F)<<8 | l;
				SPL06_calparam.c1 = (SPL06_calparam.c1&0x0800)?(0xF000|SPL06_calparam.c1):SPL06_calparam.c1;
				
				i2c_read(SPL06_ADDR, 0x13, &temp_h, 1);
				i2c_read(SPL06_ADDR, 0x14, &temp_m, 1);
				i2c_read(SPL06_ADDR, 0x15, &temp_l, 1);
			  h = temp_h;
			  m = temp_m;
			  l = temp_l;
				SPL06_calparam.c00 = (int)h<<12 | (int)m<<4 | (int)l>>4;
				SPL06_calparam.c00 = (SPL06_calparam.c00&0x080000)?(0xFFF00000|SPL06_calparam.c00):SPL06_calparam.c00;
			
				i2c_read(SPL06_ADDR, 0x15, &temp_h, 1);
				i2c_read(SPL06_ADDR, 0x16, &temp_m, 1);
				i2c_read(SPL06_ADDR, 0x17, &temp_l, 1);
				h = temp_h;
			  m = temp_m;
			  l = temp_l;
				SPL06_calparam.c10 = (int)h<<16 | (int)m<<8 | l;
				SPL06_calparam.c10 = (SPL06_calparam.c10&0x080000)?(0xFFF00000|SPL06_calparam.c10):SPL06_calparam.c10;
				
				i2c_read(SPL06_ADDR, 0x18, &temp_h, 1);
				i2c_read(SPL06_ADDR, 0x19, &temp_l, 1);
				h = temp_h;
			  l = temp_l;
				SPL06_calparam.c01 = (short)h<<8 | l;
				
				i2c_read(SPL06_ADDR, 0x1A, &temp_h, 1);
				i2c_read(SPL06_ADDR, 0x1B, &temp_l, 1);
				h = temp_h;
			  l = temp_l;
				SPL06_calparam.c11 = (short)h<<8 | l;
				
				i2c_read(SPL06_ADDR, 0x1C, &temp_h, 1);
				i2c_read(SPL06_ADDR, 0x1D, &temp_l, 1);
				h = temp_h;
			  l = temp_l;
				SPL06_calparam.c20 = (short)h<<8 | l;
				
				i2c_read(SPL06_ADDR, 0x1E, &temp_h, 1);
				i2c_read(SPL06_ADDR, 0x1F, &temp_l, 1);
				h = temp_h;
			  l = temp_l;
				SPL06_calparam.c21 = (short)h<<8 | l;
				
				i2c_read(SPL06_ADDR, 0x20, &temp_h, 1);
				i2c_read(SPL06_ADDR, 0x21, &temp_l, 1);
				h = temp_h;
			  l = temp_l;
				SPL06_calparam.c30 = (short)h<<8 | l;
	  }
}

void SPL06_height_process()
{
	HEIGHT_process_complete = 0;
	
	SPL06_info_update();
	attitude.height = SPL06_data.fRelative_Alt * 100;   //数据转存入姿态结构体，单位cm
	
	HEIGHT_process_complete = 1;
}

float SPL06_get_pressure()
{
	float fTsc = 0;
	float fPsc = 0;
	float fqua2 = 0;
	float fqua3 = 0;
	float fPCompensate = 0;

	fTsc = SPL06_data.i32RawTemperature / (float)SPL06_data.i32KT;
	fPsc = SPL06_data.i32RawPressure / (float)SPL06_data.i32KP;
	
	fqua2 = SPL06_calparam.c10 \
				 + fPsc * (SPL06_calparam.c20 \
				 + fPsc* SPL06_calparam.c30);
	fqua3 = fTsc * fPsc * (SPL06_calparam.c11 \
				 + fPsc * SPL06_calparam.c21);
	
	fPCompensate = SPL06_calparam.c00 \
								 + fPsc * fqua2 + fTsc * SPL06_calparam.c01\
								 + fqua3;
	
	return fPCompensate;
}


void SPL06_info_update()
{
	/* tropospheric properties (0-11km) for standard atmosphere */
	/* temperature at base height in Kelvin, [K] = [癈] + 273.15 */
	const double T1 = 15.0 + 273.15;

	/* temperature gradient in degrees per metre */    
	const double a = -6.5 / 1000;    
	
	/* gravity constant in m / s/s */
	const double g = 9.80665;    
	
	/* ideal gas constant in J/kg/K */
	const double R = 287.05;    
	
	/* current pressure at MSL in kPa */
	double p1 = 101325.0 / 1000.0;

	/* measured pressure in kPa */
	double p = SPL06_data.fPressure / 1000.0;

	
	SPL06_get_raw_data();    //获取原始数据
	
	SPL06_data.fPressure = SPL06_get_pressure();
	
	SPL06_data.fALT = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;
  SPL06_data.fRelative_Alt = (int16_t)((int16_t)(SPL06_data.fALT * 1000) - (int16_t)(SPL06_data.fGround_Alt * 1000))/1000.0f;
}

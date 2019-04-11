#include "Headfile.h"
#include "I2C.h"
#include "WP_DataType.h"
#include "IST8310.h"


/* ist8310 Slave Address Select : default address 0x0C
 *        CAD1  |  CAD0   |  Address
 *    ------------------------------
 *         VSS   |   VSS  |  0CH
 *         VSS   |   VDD  |  0DH
 *         VDD   |   VSS  |  0EH
 *         VDD   |   VDD  |  0FH
 * if CAD1 and CAD0 are floating, I2C address will be 0EH
 *
 *
 * CTRL_REGA: Control Register 1
 * Read Write
 * Default value: 0x0A
 * 7:4  0   Reserved.
 * 3:0  DO2-DO0: Operating mode setting
 *        DO3  |  DO2 |  DO1 |  DO0 |   mode
 *    ------------------------------------------------------
 *         0   |   0  |  0   |  0   |   Stand-By mode
 *         0   |   0  |  0   |  1   |   Single measurement mode
 *                                       Others: Reserved
 *
 * CTRL_REGB: Control Register 2
 * Read Write
 * Default value: 0x0B
 * 7:4  0   Reserved.
 * 3    DREN : Data ready enable control: 
 *      0: disable 
 *      1: enable
 * 2    DRP: DRDY pin polarity control
 *      0: active low
 *      1: active high
 * 1    0   Reserved.
 * 0    SRST: Soft reset, perform Power On Reset (POR) routine
 *      0: no action
 *      1: start immediately POR routine
 *      This bit will be set to zero after POR routine
 */
#define IST8310_SLAVE_ADDRESS     0x0E
#define IST8310_REG_STB           0x0C	//Self-Test response
#define IST8310_REG_INFO          0x01	//More Info
#define IST8310_REG_WIA           0x00	//Who I am
#define IST8310_REG_DATAX         0x03	//Output Value x
#define IST8310_REG_DATAY         0x05	//Output Value y
#define IST8310_REG_DATAZ         0x07	//Output Value z
#define IST8310_REG_STAT1         0x02	//Status register
#define IST8310_REG_STAT2         0x09	//Status register
#define IST8310_REG_CNTRL1        0x0A	//Control setting register 1
#define IST8310_REG_CNTRL2        0x0B	//Control setting register 2
#define IST8310_REG_CNTRL3        0x0D	//Control setting register 3
#define IST8310_REG_OFFSET_START  0xDC	//Offset
#define IST8310_REG_SELECTION_REG 0x42   //Sensor Selection register
#define IST8310_REG_TEST_REG      0x40   //Chip Test register
#define IST8310_REG_TUNING_REG    0x47    //Bandgap Tuning register
/*---IST8310 cross-axis matrix Address-----------------danny-----*/
#define IST8310_REG_XX_CROSS_L    0x9C  //cross axis xx low byte
#define IST8310_REG_XX_CROSS_H    0x9D  //cross axis xx high byte
#define IST8310_REG_XY_CROSS_L    0x9E  //cross axis xy low byte
#define IST8310_REG_XY_CROSS_H    0x9F  //cross axis xy high byte
#define IST8310_REG_XZ_CROSS_L    0xA0  //cross axis xz low byte
#define IST8310_REG_XZ_CROSS_H    0xA1  //cross axis xz high byte                          =       ;
#define IST8310_REG_YX_CROSS_L    0xA2  //cross axis yx low byte
#define IST8310_REG_YX_CROSS_H    0xA3  //cross axis yx high byte
#define IST8310_REG_YY_CROSS_L    0xA4  //cross axis yy low byte
#define IST8310_REG_YY_CROSS_H    0xA5  //cross axis yy high byte
#define IST8310_REG_YZ_CROSS_L    0xA6  //cross axis yz low byte
#define IST8310_REG_YZ_CROSS_H    0xA7  //cross axis yz high byte                    =       ;
#define IST8310_REG_ZX_CROSS_L    0xA8  //cross axis zx low byte
#define IST8310_REG_ZX_CROSS_H    0xA9  //cross axis zx high byte
#define IST8310_REG_ZY_CROSS_L    0xAA  //cross axis zy low byte
#define IST8310_REG_ZY_CROSS_H    0xAB  //cross axis zy high byte
#define IST8310_REG_ZZ_CROSS_L    0xAC  //cross axis zz low byte
#define IST8310_REG_ZZ_CROSS_H    0xAD  //cross axis zz high byte
#define IST8310_AXES_NUM          3


Vector2f magn={0,0};
Vector3f mag_offset;
Butter_Parameter Mag_Parameter;

void IST8310_Init(void)//磁力计初始化配置
{
  Single_WriteI2C(IST8310_SLAVE_ADDRESS,0x41,0x24);//开启16x内部平均
  Single_WriteI2C(IST8310_SLAVE_ADDRESS,0x42,0xC0);//Set/Reset内部平均
	
	Set_Cutoff_Frequency(40, 18,&Mag_Parameter);//传感器校准加计滤波值
}

uint8_t IST8310_Read_Data(vector3f *mag)//读取磁力计数据状态机
{
	static uint16_t ist8310_sampling_cnt=0;
	uint8_t buf[6];
  ist8310_sampling_cnt++;
	if(ist8310_sampling_cnt==1)
  {
	  Single_WriteI2C(IST8310_SLAVE_ADDRESS,IST8310_REG_CNTRL1,0x02);//Single Measurement Mode
	}
	else if(ist8310_sampling_cnt==10)
	{
	  buf[0]=Single_ReadI2C(IST8310_SLAVE_ADDRESS,0x03);//OUT_X_L_A
    buf[1]=Single_ReadI2C(IST8310_SLAVE_ADDRESS,0x04);//OUT_X_H_A
    buf[2]=Single_ReadI2C(IST8310_SLAVE_ADDRESS,0x05);//OUT_Y_L_A
    buf[3]=Single_ReadI2C(IST8310_SLAVE_ADDRESS,0x06);//OUT_Y_H_A
    buf[4]=Single_ReadI2C(IST8310_SLAVE_ADDRESS,0x07);//OUT_Z_L_A
    buf[5]=Single_ReadI2C(IST8310_SLAVE_ADDRESS,0x08);//OUT_Z_H_A
		mag->x=(int16_t)(buf[1]<<8|buf[0]);
		mag->y=-(int16_t)(buf[3]<<8|buf[2]);
		mag->z=(int16_t)(buf[5]<<8|buf[4]);
	  ist8310_sampling_cnt=0;
		return 1;
	}
	return 0;
}




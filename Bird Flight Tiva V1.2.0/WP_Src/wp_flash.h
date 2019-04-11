#ifndef __WP_FLASH_H__
#define __WP_FLASH_H__

void EEPROM_Init(void);


#define PARAMETER_TABLE_STARTADDR   0x0000 
#define FLIGHT_PARAMETER_TABLE_NUM  100
typedef struct
{
  float Parameter_Table[FLIGHT_PARAMETER_TABLE_NUM];
}FLIGHT_PARAMETER;


typedef enum
{
  PITCH_OFFSET=0,
  ROLL_OFFSET=1,
  ACCEL_X_OFFSET=2,
  ACCEL_Y_OFFSET=3,
  ACCEL_Z_OFFSET=4,
  ACCEL_X_SCALE=5,
  ACCEL_Y_SCALE=6,
  ACCEL_Z_SCALE=7,
  MAG_X_OFFSET=8,
  MAG_Y_OFFSET=9,
  MAG_Z_OFFSET=10,
  RC_CH1_MAX=11,
  RC_CH1_MIN=12,
  RC_CH2_MAX=13,
  RC_CH2_MIN=14,
  RC_CH3_MAX=15,
  RC_CH3_MIN=16,
  RC_CH4_MAX=17,
  RC_CH4_MIN=18,
  RC_CH5_MAX=19,
  RC_CH5_MIN=20,
  RC_CH6_MAX=21,
  RC_CH6_MIN=22,
  RC_CH7_MAX=23,
  RC_CH7_MIN=24,
  RC_CH8_MAX=25,
  RC_CH8_MIN=26,
  PID1_PARAMETER_KP=27,
  PID1_PARAMETER_KI=28,
  PID1_PARAMETER_KD=29,
  PID2_PARAMETER_KP=30,
  PID2_PARAMETER_KI=31,
  PID2_PARAMETER_KD=32,
  PID3_PARAMETER_KP=33,
  PID3_PARAMETER_KI=34,
  PID3_PARAMETER_KD=35,
  PID4_PARAMETER_KP=36,
  PID4_PARAMETER_KI=37,
  PID4_PARAMETER_KD=38,
  PID5_PARAMETER_KP=39,
  PID5_PARAMETER_KI=40,
  PID5_PARAMETER_KD=41,
  PID6_PARAMETER_KP=42,
  PID6_PARAMETER_KI=43,
  PID6_PARAMETER_KD=44,
  PID7_PARAMETER_KP=45,
  PID7_PARAMETER_KI=46,
  PID7_PARAMETER_KD=47,
  PID8_PARAMETER_KP=48,
  PID8_PARAMETER_KI=49,
  PID8_PARAMETER_KD=50,
  PID9_PARAMETER_KP=51,
  PID9_PARAMETER_KI=52,
  PID9_PARAMETER_KD=53,
  PID10_PARAMETER_KP=54,
  PID10_PARAMETER_KI=55,
  PID10_PARAMETER_KD=56,
  PID11_PARAMETER_KP=57,
  PID11_PARAMETER_KI=58,
  PID11_PARAMETER_KD=59,
  PID12_PARAMETER_KP=60,
  PID12_PARAMETER_KI=61,
  PID12_PARAMETER_KD=62,
  PID13_PARAMETER_KP=63,
  PID13_PARAMETER_KI=64,
  PID13_PARAMETER_KD=65,
  PID14_PARAMETER_KP=66,
  PID14_PARAMETER_KI=67,
  PID14_PARAMETER_KD=68,
  PID15_PARAMETER_KP=69,
  PID15_PARAMETER_KI=70,
  PID15_PARAMETER_KD=71,
  PID16_PARAMETER_KP=72,
  PID16_PARAMETER_KI=73,
  PID16_PARAMETER_KD=74,   
  PID17_PARAMETER_KP=75,
  PID17_PARAMETER_KI=76,
  PID17_PARAMETER_KD=77,
  PID18_PARAMETER_KP=78,
  PID18_PARAMETER_KI=79,
  PID18_PARAMETER_KD=80,   
  PID19_PARAMETER_KP=81,
  PID19_PARAMETER_KI=82,
  PID19_PARAMETER_KD=83,
  ESC_CALIBRATION_FLAG=84,
  HEADLESS_MODE_YAW=85,
  HOR_CAL_ACCEL_X=86,
  HOR_CAL_ACCEL_Y=87,
  HOR_CAL_ACCEL_Z=88,
  SDK_MODE_DEFAULT=89
}FLIGHT_PARAMETER_TABLE;

extern FLIGHT_PARAMETER Table_Parameter;

uint32_t STMFLASH_ReadWord(uint32_t faddr);		  	//读出字  
void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead);   		//从指定地址开始读出指定长度的数据

void ReadFlashParameterALL(FLIGHT_PARAMETER *WriteData);
void ReadFlashParameterOne(uint16_t Label,float *ReadData);
void ReadFlashParameterTwo(uint16_t Label,float *ReadData1,float *ReadData2);
void ReadFlashParameterThree(uint16_t Label,float *ReadData1,float *ReadData2,float *ReadData3);


void WriteFlashParameter(uint16_t Label,float WriteData,FLIGHT_PARAMETER *Table);
void WriteFlashParameter_Two(uint16_t Label,
                             float WriteData1,
                             float WriteData2,
                             FLIGHT_PARAMETER *Table);
void WriteFlashParameter_Three(uint16_t Label,
                               float WriteData1,
                               float WriteData2,
                               float WriteData3,
                               FLIGHT_PARAMETER *Table);
void ReadFlashParameterTwo(uint16_t Label,float *ReadData1,float *ReadData2);

#endif


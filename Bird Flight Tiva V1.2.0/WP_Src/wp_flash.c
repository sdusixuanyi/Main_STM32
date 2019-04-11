#include "Headfile.h"

#include "eeprom.h"
#include "flash.h"
#include "sysctl.h"
#include "wp_flash.h"

//FLASH起始地址
#define WP_FLASH_BASE PARAMETER_TABLE_STARTADDR 	//STM32 FLASH的起始地址

FLIGHT_PARAMETER Table_Parameter;
/*
struct E2PROM
{
uint8_t value1;
uint8_t value2;
uint16_t value3;
uint8_t value4[12];
}; 
*/
//struct E2PROM e2prom_write_value = {5,7,9826, "Hello World"};
//struct E2PROM e2prom_read_value =  {0,0,0,""};


void EEPROM_Init(void)
{
  /* EEPROM SETTINGS */
  SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0); // EEPROM activate
  EEPROMInit(); // EEPROM start
}

void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)	
{ 
  EEPROMProgram(pBuffer,WriteAddr,NumToWrite);
}

void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
{
  EEPROMRead(pBuffer,ReadAddr,NumToRead);
}


void ReadFlashParameterALL(FLIGHT_PARAMETER *WriteData)
{
  EEPROMRead((uint32_t *)(&WriteData->Parameter_Table),WP_FLASH_BASE,FLIGHT_PARAMETER_TABLE_NUM*4);
}


void ReadFlashParameterOne(uint16_t Label,float *ReadData)
{
  EEPROMRead((uint32_t *)(ReadData),WP_FLASH_BASE+4*Label,4);
}

void ReadFlashParameterTwo(uint16_t Label,float *ReadData1,float *ReadData2)
{
  EEPROMRead((uint32_t *)(ReadData1),WP_FLASH_BASE+4*Label,4);;
  EEPROMRead((uint32_t *)(ReadData2),WP_FLASH_BASE+4*Label+4,4);
}

void ReadFlashParameterThree(uint16_t Label,float *ReadData1,float *ReadData2,float *ReadData3)
{
  EEPROMRead((uint32_t *)(ReadData1),WP_FLASH_BASE+4*Label,4);;
  EEPROMRead((uint32_t *)(ReadData2),WP_FLASH_BASE+4*Label+4,4);
  EEPROMRead((uint32_t *)(ReadData3),WP_FLASH_BASE+4*Label+8,4);
}

void WriteFlashParameter(uint16_t Label,
                         float WriteData,
                         FLIGHT_PARAMETER *Table)
{
  ReadFlashParameterALL(Table);//先把片区内的所有数据都都出来值
  Table->Parameter_Table[Label]=WriteData;//将需要更改的字段赋新
  EEPROMProgram((uint32_t *)(&Table->Parameter_Table[Label]),WP_FLASH_BASE+4*Label,4);
}

void WriteFlashParameter_Two(uint16_t Label,
                             float WriteData1,
                             float WriteData2,
                             FLIGHT_PARAMETER *Table)
{
  ReadFlashParameterALL(Table);//先把片区内的所有数据都都出来
  Table->Parameter_Table[Label]=WriteData1;//将需要更改的字段赋新值
  Table->Parameter_Table[Label+1]=WriteData2;//将需要更改的字段赋新值
  EEPROMProgram((uint32_t *)(&Table->Parameter_Table[Label]),WP_FLASH_BASE+4*Label,8);
}

void WriteFlashParameter_Three(uint16_t Label,
                               float WriteData1,
                               float WriteData2,
                               float WriteData3,
                               FLIGHT_PARAMETER *Table)
{
  ReadFlashParameterALL(Table);//先把片区内的所有数据都都出来
  Table->Parameter_Table[Label]=WriteData1;//将需要更改的字段赋新值
  Table->Parameter_Table[Label+1]=WriteData2;//将需要更改的字段赋新值
  Table->Parameter_Table[Label+2]=WriteData3;//将需要更改的字段赋新值
  EEPROMProgram((uint32_t *)(&Table->Parameter_Table[Label]),WP_FLASH_BASE+4*Label,12);
}


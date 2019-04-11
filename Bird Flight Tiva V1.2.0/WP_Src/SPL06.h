#ifndef __SPL06_H
#define __SPL06_H


void spl0601_start_continuous(uint8_t mode);
void spl0601_rateset(uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl);

void SPL06_Init(void);
void SPL06_Read_Data(float *baro_t,float *baro_p);
#endif


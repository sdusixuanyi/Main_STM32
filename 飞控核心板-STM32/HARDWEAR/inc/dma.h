#ifndef _DMA_H_
#define _DMA_H_

//#include "stm32f10x.h"
#include "headfile.h"


#define I2C1_DR_ADDR  0x40005410    //i2c�������ݼĴ���
#define I2C2_DR_ADDR  0x40005810

void dma_init(void);

void dma_nvic_init(void);

#endif

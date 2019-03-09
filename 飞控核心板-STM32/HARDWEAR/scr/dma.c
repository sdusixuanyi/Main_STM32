#include "dma.h"

extern unsigned char raw_data[14];

extern long int cycle;

int dma_flag = 1;
void dma_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//开启DMA时钟
  //DMA_DeInit(DMA1_Channel7);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = I2C2_DR_ADDR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)raw_data;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 14;                        //非循环模式下每次都需要重设该值！
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	
	dma_nvic_init();
	
	DMA_Cmd (DMA1_Channel5,ENABLE);					//使能DMA
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);             //传输完成后触发中断
}

void dma_nvic_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel=DMA1_Channel5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitStructure);
}

void DMA1_Channel5_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC5) == SET)
	{
		DMA_ClearFlag(DMA1_FLAG_TC5);      //清中断标志
		
		I2C_DMACmd(I2C2, DISABLE);
		            
    /* Send I2C1 STOP Condition */
    I2C_GenerateSTOP(I2C2, ENABLE);
    /* Disable DMA channel*/
    //DMA_Cmd(DMA1_Channel7, DISABLE);

    //MPU6050_data_process();
		dma_flag = 1;
	}
}

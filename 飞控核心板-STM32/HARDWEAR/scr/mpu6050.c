#include "mpu6050.h"

extern unsigned char raw_data[14];
extern short int translated_data[7];     //将6050的原始14个数据化为7个二进制数据，前三个是加速度分别在xyz上的分量，最后三个是角速度，中间的是温度

void MPU6050_init(void)
{
	i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);         //reg107,唤醒，8M内部时钟源
	i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x07);         //采用频率  1000
	i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x06);
	i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x09);       //加速度量程4g
	i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x18);        //角速度量程2000°/s
	
	while(!MPU6050_check());
	MPU6050_get_offset();
}

void MPU6050_get_data(unsigned char* addr)
{
	
	i2c_read(MPU6050_ADDRESS, 0x3B, addr, 14);
	//USART_SendData(USART1, *addr);
	//USART_SendData(USART1, '\n');
}

void MPU6050_data_translation(unsigned char* raw_addr, short int* translation_addr)
{
	int i;
	for(i = 0 ; i < 7 ; i++)
	{
		*(translation_addr + i) = ((short int)(*(raw_addr + 2*i)) << 8) + (short int)(*(raw_addr + 1 + 2*i));		
		//translated_data[i] = (raw_data[2*i] << 8) + raw_data[2*i + 1];
	}
	
// 	for(i = 0 ; i < 3 ; i ++)
// 	{
// 		if(translated_data[i] >= 0)  accel[i] = ((float)translated_data[i] / (float)32767) * ACCEL_RANGE;        //加速度换算
// 		else                         accel[i] = ((float)translated_data[i] / (float)32768) * ACCEL_RANGE;
// 		
// 		if(translated_data[i + 4] >= 0)   gyro[i] = (0 - ((float)translated_data[i + 4] / (float)32767) * GYRO_RANGE) - Sta_gyro[i];   //角速度换算
// 		else                              gyro[i] = (0 - ((float)translated_data[i + 4] / (float)32768) * GYRO_RANGE) - Sta_gyro[i];		
// 	}
	
	
}



void MPU6050_dma_read(unsigned char slave_addr, unsigned char reg_addr)
{
	//static int count = 0;
	int wait_num = 0;
	//int i;
	//GPIO_Write(GPIOF, 0xff00);
	
// 	if(count == 0)
// 	{
// 		printf("count = 0\n");
// 		printf("CR1 = %x\n", I2C_ReadRegister(I2C1, I2C_Register_CR1));
// 		printf("CR2 = %x\n", I2C_ReadRegister(I2C1, I2C_Register_CR2));
// 	  printf("SR1 = %x\n", I2C_ReadRegister(I2C1, I2C_Register_SR1));
// 		printf("SR2 = %x\n\n", I2C_ReadRegister(I2C1, I2C_Register_SR2));
// 		for(i = 0 ; i <= 13 ; i++)
// 		{
// 			printf("raw[i]= %d ", raw_data[i]);
// 		}
// 		printf("\n\n");
// 	}
	
	
	
  DMA_Cmd(DMA1_Channel7, DISABLE);               //关闭I2C的DMA通道	
  DMA_SetCurrDataCounter(DMA1_Channel7, 14);     //重设传输数14，必须在关闭DMA相应通道后才能设，在非循环模式下每次传输完成后该值将归0！
	
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))    //等待空闲
	{
		wait_num++;

		if(wait_num > 5000)
		{			
// 			if(usart1_permission)
// 			{
// 				printf("count = %d:  bus locked busy\n", count);
// 			  printf("CR1 = %x\n", I2C_ReadRegister(I2C1, I2C_Register_CR1));
// 		    printf("CR2 = %x\n", I2C_ReadRegister(I2C1, I2C_Register_CR2));
// 		    printf("SR1 = %x\n", I2C_ReadRegister(I2C1, I2C_Register_SR1));
// 		    printf("SR2= %x\n\n", I2C_ReadRegister(I2C1, I2C_Register_SR2));
// 			}
			return;
		}
	}

  //printf("count: %d  pass on!!\n", count);
	
	
  I2C_DMALastTransferCmd(I2C1, ENABLE);                    //原注释：Note this one, very important   编写者：不明所以的操作(貌似自动产生nack信号)
	
  I2C_GenerateSTART(I2C1, ENABLE);                                //发送开始信号
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));     //EV5

  I2C_Send7bitAddress(I2C1, slave_addr, I2C_Direction_Transmitter);     //发送从机地址，发送者模式
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  //EV6

  I2C_Cmd(I2C1, ENABLE);                                               //用重新使能的方式清空EV6

  I2C_SendData(I2C1, reg_addr);                                           //读取的地址
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));    //EV8

  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));         //EV5

  I2C_Send7bitAddress(I2C1, slave_addr, I2C_Direction_Receiver);       //发送从机地址，接收模式
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));    //EV6

  /* Start DMA to receive data from I2C */
  DMA_Cmd(DMA1_Channel7, ENABLE);
  I2C_DMACmd(I2C1, ENABLE);
	
	//if(count == 0 || usart1_permission)  printf("count = %d  I2C request out\n\n", count);
	
	//count ++;
	
	

  // When the data transmission is complete, it will automatically jump to DMA interrupt routine to finish the rest.
  //now go back to the main routine
}

int MPU6050_check()
{
	long long int count;
	int i, j;
	
	for(i = 0 ; i < 100 ; i ++)
	{	
		MPU6050_get_data(raw_data);      //常规读取
		
		MPU6050_data_translation(raw_data, translated_data);
		
		for(j = 4 ; j <= 6 ; j++)    
		{
			translated_data[i] = translated_data[i] * Gyro_G;   //将角速度化为十进制
			count += translated_data[i];
		}
	}
	
	if ( (count / 300) < 5 ) return 1;
	else                     return 0;
}

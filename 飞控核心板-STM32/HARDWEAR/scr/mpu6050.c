#include "mpu6050.h"

extern unsigned char raw_data[14];
extern short int translated_data[7];     //��6050��ԭʼ14�����ݻ�Ϊ7�����������ݣ�ǰ�����Ǽ��ٶȷֱ���xyz�ϵķ�������������ǽ��ٶȣ��м�����¶�

void MPU6050_init(void)
{
	i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);         //reg107,���ѣ�8M�ڲ�ʱ��Դ
	i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x07);         //����Ƶ��  1000
	i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x06);
	i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x09);       //���ٶ�����4g
	i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x18);        //���ٶ�����2000��/s
	
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
// 		if(translated_data[i] >= 0)  accel[i] = ((float)translated_data[i] / (float)32767) * ACCEL_RANGE;        //���ٶȻ���
// 		else                         accel[i] = ((float)translated_data[i] / (float)32768) * ACCEL_RANGE;
// 		
// 		if(translated_data[i + 4] >= 0)   gyro[i] = (0 - ((float)translated_data[i + 4] / (float)32767) * GYRO_RANGE) - Sta_gyro[i];   //���ٶȻ���
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
	
	
	
  DMA_Cmd(DMA1_Channel7, DISABLE);               //�ر�I2C��DMAͨ��	
  DMA_SetCurrDataCounter(DMA1_Channel7, 14);     //���贫����14�������ڹر�DMA��Ӧͨ��������裬�ڷ�ѭ��ģʽ��ÿ�δ�����ɺ��ֵ����0��
	
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))    //�ȴ�����
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
	
	
  I2C_DMALastTransferCmd(I2C1, ENABLE);                    //ԭע�ͣ�Note this one, very important   ��д�ߣ��������ԵĲ���(ò���Զ�����nack�ź�)
	
  I2C_GenerateSTART(I2C1, ENABLE);                                //���Ϳ�ʼ�ź�
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));     //EV5

  I2C_Send7bitAddress(I2C1, slave_addr, I2C_Direction_Transmitter);     //���ʹӻ���ַ��������ģʽ
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  //EV6

  I2C_Cmd(I2C1, ENABLE);                                               //������ʹ�ܵķ�ʽ���EV6

  I2C_SendData(I2C1, reg_addr);                                           //��ȡ�ĵ�ַ
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));    //EV8

  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));         //EV5

  I2C_Send7bitAddress(I2C1, slave_addr, I2C_Direction_Receiver);       //���ʹӻ���ַ������ģʽ
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
		MPU6050_get_data(raw_data);      //�����ȡ
		
		MPU6050_data_translation(raw_data, translated_data);
		
		for(j = 4 ; j <= 6 ; j++)    
		{
			translated_data[i] = translated_data[i] * Gyro_G;   //�����ٶȻ�Ϊʮ����
			count += translated_data[i];
		}
	}
	
	if ( (count / 300) < 5 ) return 1;
	else                     return 0;
}

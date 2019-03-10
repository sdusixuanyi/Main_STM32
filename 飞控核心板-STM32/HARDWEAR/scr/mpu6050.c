#include "mpu6050.h"

extern unsigned char raw_data[14];
extern short int translated_data[7];     //��6050��ԭʼ14�����ݻ�Ϊ7�����������ݣ�ǰ�����Ǽ��ٶȷֱ���xyz�ϵķ�������������ǽ��ٶȣ��м�����¶�

extern int dma_flag;
extern float accel[3];         
extern float gyro[3];
float data[7] = {0};

void MPU6050_init(int mode)
{
	if(mode == MPU6050_HARDWARE)
	{
		i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);         //reg107,���ѣ�8M�ڲ�ʱ��Դ
		i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x07);         //����Ƶ��  1000
		i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x06);
		i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x09);       //���ٶ�����4g
		i2c_byte_write(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x18);        //���ٶ�����2000��/s
	}
	else
	{
		i2c_byte_write_soft(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);         //reg107,���ѣ�8M�ڲ�ʱ��Դ
		i2c_byte_write_soft(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x07);         //����Ƶ��  1000
		i2c_byte_write_soft(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x06);
		i2c_byte_write_soft(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x09);       //���ٶ�����4g
		i2c_byte_write_soft(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x18);        //���ٶ�����2000��/s
	}
	
	while(!MPU6050_check(mode));
	MPU6050_get_offset(mode);
}

void MPU6050_get_data(unsigned char* addr, int mode)
{
	
	if(mode == MPU6050_HARDWARE) i2c_read(MPU6050_ADDRESS, 0x3B, addr, 14);
	else                         i2c_read_soft(MPU6050_ADDRESS, 0x3B, addr, 14);
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
	
 	for(i = 0 ; i < 3 ; i ++)
 	{
 		if(translated_data[i] >= 0)  accel[i] = ((float)translated_data[i] / (float)32767) * ACCEL_RANGE;        //���ٶȻ���
 		else                         accel[i] = ((float)translated_data[i] / (float)32768) * ACCEL_RANGE;
 		
 		if(translated_data[i + 4] >= 0)   gyro[i] = (0 - ((float)translated_data[i + 4] / (float)32767) * GYRO_RANGE);   //���ٶȻ���
 		else                              gyro[i] = (0 - ((float)translated_data[i + 4] / (float)32768) * GYRO_RANGE);		
 	}
	
	
}



void MPU6050_dma_read(unsigned char slave_addr, unsigned char reg_addr)
{
	//static int count = 0;
	int wait_num = 0;

	
	
  DMA_Cmd(DMA1_Channel5, DISABLE);               //�ر�I2C��DMAͨ��	
  DMA_SetCurrDataCounter(DMA1_Channel5, 14);     //���贫����14�������ڹر�DMA��Ӧͨ��������裬�ڷ�ѭ��ģʽ��ÿ�δ�����ɺ��ֵ����0��
	
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))    //�ȴ�����
	{
		wait_num++;

		if(wait_num > 5000)
		{			
			return;
		}
	}

  //printf("count: %d  pass on!!\n", count);
	
	
  I2C_DMALastTransferCmd(I2C2, ENABLE);                    //ԭע�ͣ�Note this one, very important   ��д�ߣ��������ԵĲ���(ò���Զ�����nack�ź�)
	
  I2C_GenerateSTART(I2C2, ENABLE);                                //���Ϳ�ʼ�ź�
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));     //EV5

  I2C_Send7bitAddress(I2C2, slave_addr, I2C_Direction_Transmitter);     //���ʹӻ���ַ��������ģʽ
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  //EV6

  I2C_Cmd(I2C2, ENABLE);                                               //������ʹ�ܵķ�ʽ���EV6

  I2C_SendData(I2C2, reg_addr);                                           //��ȡ�ĵ�ַ
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));    //EV8

  I2C_GenerateSTART(I2C2, ENABLE);
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));         //EV5

  I2C_Send7bitAddress(I2C2, slave_addr, I2C_Direction_Receiver);       //���ʹӻ���ַ������ģʽ
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));    //EV6

  /* Start DMA to receive data from I2C */
  DMA_Cmd(DMA1_Channel5, ENABLE);
  I2C_DMACmd(I2C2, ENABLE);
	dma_flag = 0;
	//if(count == 0 || usart1_permission)  printf("count = %d  I2C request out\n\n", count);
	
	//count ++;
	
	

  // When the data transmission is complete, it will automatically jump to DMA interrupt routine to finish the rest.
  //now go back to the main routine
}

int MPU6050_check(int mode)
{
	long long int count = 0;
	int i, j;
	
	for(i = 0 ; i < 100 ; i ++)
	{	
		MPU6050_get_data(raw_data, mode);      //�����ȡ
		
		MPU6050_data_translation(raw_data, translated_data);
		
//		for(j = 0 ; j <= 2 ; j++)    
//		{
//			data[j] = (float)translated_data[j] * 4 / 32768;   //�����ٶȻ�Ϊʮ����
//			count += 0;//translated_data[i];
//		}
		
		for(j = 4 ; j <= 6 ; j++)    
		{
			data[j] = (float)translated_data[j] * Gyro_G;   //�����ٶȻ�Ϊʮ����
			count += data[j];
		}
	}
	
	if ( (count / 300) < 5 ) return 1;
	else                     return 0;
}

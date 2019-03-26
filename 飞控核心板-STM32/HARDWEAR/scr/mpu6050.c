#include "mpu6050.h"

extern unsigned char raw_data[14];
extern short int translated_data[7];     //��6050��ԭʼ14�����ݻ�Ϊ7�����������ݣ�ǰ�����Ǽ��ٶȷֱ���xyz�ϵķ�������������ǽ��ٶȣ��м�����¶�

extern int dma_flag;
extern float accel[3];         
extern float gyro[3];
extern struct Attitude_float attitude;
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
	else if(mode == MPU6050_SOFTWARE)
	{
		i2c_byte_write_soft(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);         //reg107,���ѣ�8M�ڲ�ʱ��Դ
		i2c_byte_write_soft(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x07);         //����Ƶ��  1000
		i2c_byte_write_soft(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x06);
		i2c_byte_write_soft(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x09);       //���ٶ�����4g
		i2c_byte_write_soft(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x18);        //���ٶ�����2000��/s
	}
	else if(mode == MPU6050_DMP)
	{
		MPU_Init();
		while(mpu_dmp_init());
	}
	
	while(!MPU6050_check(mode));
	MPU6050_get_offset(mode);
}

void MPU6050_get_data(unsigned char* addr, u8 mode)
{	
	if(mode == MPU6050_HARDWARE) i2c_read(MPU6050_ADDRESS, 0x3B, addr, 14);
	else if(mode == MPU6050_SOFTWARE)   i2c_read_soft(MPU6050_ADDRESS, 0x3B, addr, 14);
	else if((mode == MPU6050_DMP))     MPU6050_DMP_read(mode, addr);

}

void MPU6050_data_translation(unsigned char* raw_addr, short int* translation_addr, u8 mode)
{
	int i;
	
	if(mode != MPU6050_DMP)
	{
		for(i = 0 ; i < 7 ; i++)
		{
			*(translation_addr + i) = ((short int)(*(raw_addr + 2*i)) << 8) + (short int)(*(raw_addr + 1 + 2*i));		
			//translated_data[i] = (raw_data[2*i] << 8) + raw_data[2*i + 1];
		}
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
//    switch(mode)
//		{
//			case MPU6050_HARDWARE : MPU6050_get_data(raw_data, mode); break;
//			case MPU6050_SOFTWARE : MPU6050_get_data(raw_data, mode); break;
//			case MPU6050_DMPwithoutRAWDATA : MPU6050_get_data(raw_data, MPU6050_SOFTWARE); break;
//			case MPU6050_DMPwithRAWDATA    : MPU6050_get_data(raw_data, MPU6050_SOFTWARE); break;
//			default : break;
//		}
    		
		MPU6050_get_data(raw_data, mode);      //�����ȡ
		
		MPU6050_data_translation(raw_data, translated_data, mode);
		
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

void MPU6050_DMP_read(u8 IfGetRawData, unsigned char* addr)
{
	mpu_dmp_get_data(&attitude.pitch, &attitude.roll, &attitude.yaw);
	
//	if(IfGetRawData == MPU6050_DMP)
//	{
//		i2c_read_soft(MPU6050_ADDRESS, 0x3B, addr, 14);
//	}
}



//***************************************************����ԭ��DMP������ֲ*****************************************************
u8 MPU_Init(void)
{ 
	u8 res; 
	//MPU_IIC_Init();//��ʼ��IIC����
	//MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
	i2c_byte_write_soft(MPU6050_ADDRESS, MPU_PWR_MGMT1_REG, 0x80);
	
    delay_ms(100);
	//MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050
	i2c_byte_write_soft(MPU6050_ADDRESS, MPU_PWR_MGMT1_REG, 0x00);
	
	//MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
	i2c_byte_write_soft(MPU6050_ADDRESS, MPU_GYRO_CFG_REG, 0x18);
	
	//MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g   
	i2c_byte_write_soft(MPU6050_ADDRESS, MPU_ACCEL_CFG_REG, 0x00);
	
	MPU_Set_Rate(50);						//���ò�����50Hz
	
	//MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
	i2c_byte_write_soft(MPU6050_ADDRESS, MPU_INT_EN_REG, 0x00);
	
	//MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	i2c_byte_write_soft(MPU6050_ADDRESS, MPU_USER_CTRL_REG, 0x00);
	
	//MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	i2c_byte_write_soft(MPU6050_ADDRESS, MPU_FIFO_EN_REG, 0x00);
	
	//MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	i2c_byte_write_soft(MPU6050_ADDRESS, MPU_INTBP_CFG_REG, 0x80);
	
	//res=MPU_Read_Byte(MPU_DEVICE_ID_REG); 
	i2c_read_soft(MPU6050_ADDRESS, MPU_DEVICE_ID_REG, &res, 1);
	
	if(res==MPU_ADDR)//����ID��ȷ
	{
		//MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		i2c_byte_write_soft(MPU6050_ADDRESS, MPU_PWR_MGMT1_REG, 0x01);
		
		//MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		i2c_byte_write_soft(MPU6050_ADDRESS, MPU_PWR_MGMT2_REG, 0x00);
		
		MPU_Set_Rate(50);						//���ò�����Ϊ50Hz
 	}else return 1;
	return 0;
}


u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	//data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���    ����Ϊ����ԭ���룬����һ�����Լ�д��
	i2c_byte_write_soft(MPU6050_ADDRESS, MPU_SAMPLE_RATE_REG, data);
	
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	//return MPU_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���   ����Ϊ����ԭ���룬�����������Լ�д��
	i2c_byte_write_soft(MPU6050_ADDRESS, MPU_CFG_REG, data);
	return 1;
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//��������
		if(MPU_IIC_Wait_Ack())		//�ȴ�ACK
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop();	 
	return 0;	
} 
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte(0);//������,����nACK 
		else *buf=MPU_IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop();	//����һ��ֹͣ���� 
	return 0;	
}
//***************************************************************************************************************************

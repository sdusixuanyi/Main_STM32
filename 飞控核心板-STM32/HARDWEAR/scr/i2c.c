#include "i2c.h"


//***********************************************Ӳ��IIC����*****************************************************************
void i2c_init(int mode)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	I2C_InitTypeDef  I2C_InitStructure; 
  
	if(mode == I2C_HARDWARE)
	{
		/* ʹ���� I2C2 �йص�ʱ�� */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);  
			
		/* PB10-I2C2_SCL��PB11-I2C2_SDA*/
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;	       // ��©���
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		
		I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
		I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
		I2C_InitStructure.I2C_OwnAddress1 =I2C2_OWN_ADDRESS; 
		I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;
		I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
		I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
		I2C_Init(I2C2, &I2C_InitStructure);
		
		I2C_Cmd(I2C2, ENABLE);                  //ʹ��
	}
	else
	{
		RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	 
		IIC_SCL=1;
		IIC_SDA=1;
	}
}



void i2c_byte_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char data)
{
	I2C_GenerateSTART(I2C2, ENABLE);    //���Ϳ�ʼ�ź�
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(I2C2, slave_addr, I2C_Direction_Transmitter);    //���ʹӻ���ַ������Ϊд��
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
		
	I2C_SendData(I2C2, reg_addr);    //����д��ĵ�ַ
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	I2C_SendData(I2C2, data);         //����д�������
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	I2C_GenerateSTOP(I2C2, ENABLE);      //����
}

void i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char* data, int count)
{ 
	//static int count1 = 0;         //����
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))         //�ȴ�����
	{
		//GPIO_Write(GPIOF, 0x0f0f);
	}

  I2C_GenerateSTART(I2C2, ENABLE);                             //���Ϳ�ʼ�ź�
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));  //EV5

  I2C_Send7bitAddress(I2C2, slave_addr, I2C_Direction_Transmitter);            //�ӻ���ַ,����ģʽ
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));   //EV6

  I2C_Cmd(I2C2, ENABLE);                                         //������ʹ�ܵķ�ʽ���EV6
	
  I2C_SendData(I2C2, reg_addr);                                       //��ȡ�ĵ�ַ
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));    //EV8

  I2C_GenerateSTART(I2C2, ENABLE);                               //�ٴη��Ϳ�ʼ�ź�
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));     //EV5

  I2C_Send7bitAddress(I2C2, slave_addr, I2C_Direction_Receiver);    //���ʹӻ���ַ������ģʽ
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));   //EV6
	


  while(count)
  {
    if(count == 1)
    {
      I2C_AcknowledgeConfig(I2C2, DISABLE);            //����Ӧ������ź�

      I2C_GenerateSTOP(I2C2, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the MPU6050 */
      *data = I2C_ReceiveData(I2C2);

      /* Point to the next location where the byte read will be saved */
      data++;

      /* Decrement the read bytes counter */
      count--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C2, ENABLE);
}
//***************************************************************************************************************************




//************************************************ģ��IIC����****************************************************************
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//��ʹ������IO PORTCʱ�� 
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	IIC_SCL=1;
	IIC_SDA=1;

}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

u8 slave_addr_process(u8 slave_addr)
{
	if (slave_addr == MPU6050_ADDRESS)  slave_addr = slave_addr;
	else if(slave_addr == SPL06_ADDR)   slave_addr = slave_addr << 1;
	return slave_addr;
}

void i2c_byte_write_soft(unsigned char slave_addr, unsigned char reg_addr, unsigned char data)
{
	//slave_addr = slave_addr_process(slave_addr);
	
	IIC_Start();
	
	IIC_Send_Byte(slave_addr & 0xfe);   //�ӻ���ַ
	
	IIC_Wait_Ack();
	
	IIC_Send_Byte(reg_addr);
	
	IIC_Wait_Ack();
	
	IIC_Send_Byte(data);
	
	IIC_Wait_Ack();
	
	IIC_Stop();	
}

void i2c_read_soft(unsigned char slave_addr, unsigned char reg_addr, unsigned char* data, int count)
{
	//slave_addr = slave_addr_process(slave_addr);
	
	IIC_Start();
	
	IIC_Send_Byte(slave_addr & 0xfe);   //�ӻ���ַ
	
	IIC_Wait_Ack();
	
	IIC_Send_Byte(reg_addr);
	
	IIC_Wait_Ack();
	
	IIC_Start();
	
	IIC_Send_Byte(slave_addr | 0x01);   //�ӻ���ַ
	
	IIC_Wait_Ack();
	
	while(count >= 1)
	{
		if(count == 1)    *data = IIC_Read_Byte(0);
		else              *data = IIC_Read_Byte(1);
		
		data ++;  count--;
	}
	
	IIC_Stop();
}
//***************************************************************************************************************************


//**********************************************��ֲ������ԭ��IIC����********************************************************
void MPU_IIC_Delay(void)
{
	delay_us(2);
}


//����IIC��ʼ�ź�
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();     //sda�����
	MPU_IIC_SDA=1;	  	  
	MPU_IIC_SCL=1;
	MPU_IIC_Delay();
 	MPU_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay();
	MPU_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();//sda�����
	MPU_IIC_SCL=0;
	MPU_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay();
	MPU_IIC_SCL=1;  
	MPU_IIC_SDA=1;//����I2C���߽����ź�
	MPU_IIC_Delay();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN();      //SDA����Ϊ����  
	MPU_IIC_SDA=1;MPU_IIC_Delay();	   
	MPU_IIC_SCL=1;MPU_IIC_Delay();	 
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL=0;
	MPU_SDA_OUT();
	MPU_IIC_SDA=0;
	MPU_IIC_Delay();
	MPU_IIC_SCL=1;
	MPU_IIC_Delay();
	MPU_IIC_SCL=0;
}
//������ACKӦ��		    
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL=0;
	MPU_SDA_OUT();
	MPU_IIC_SDA=1;
	MPU_IIC_Delay();
	MPU_IIC_SCL=1;
	MPU_IIC_Delay();
	MPU_IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void MPU_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        MPU_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		MPU_IIC_SCL=1;
		MPU_IIC_Delay(); 
		MPU_IIC_SCL=0;	
		MPU_IIC_Delay();
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        MPU_IIC_SCL=0; 
        MPU_IIC_Delay();
		MPU_IIC_SCL=1;
        receive<<=1;
        if(MPU_READ_SDA)receive++;   
		MPU_IIC_Delay(); 
    }					 
    if (!ack)
        MPU_IIC_NAck();//����nACK
    else
        MPU_IIC_Ack(); //����ACK   
    return receive;
}
//***************************************************************************************************************************

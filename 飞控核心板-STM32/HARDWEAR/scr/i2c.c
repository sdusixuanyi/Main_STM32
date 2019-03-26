#include "i2c.h"


//***********************************************硬件IIC程序*****************************************************************
void i2c_init(int mode)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	I2C_InitTypeDef  I2C_InitStructure; 
  
	if(mode == I2C_HARDWARE)
	{
		/* 使能与 I2C2 有关的时钟 */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);  
			
		/* PB10-I2C2_SCL、PB11-I2C2_SDA*/
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;	       // 开漏输出
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		
		I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
		I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
		I2C_InitStructure.I2C_OwnAddress1 =I2C2_OWN_ADDRESS; 
		I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;
		I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
		I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
		I2C_Init(I2C2, &I2C_InitStructure);
		
		I2C_Cmd(I2C2, ENABLE);                  //使能
	}
	else
	{
		RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	 
		IIC_SCL=1;
		IIC_SDA=1;
	}
}



void i2c_byte_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char data)
{
	I2C_GenerateSTART(I2C2, ENABLE);    //发送开始信号
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(I2C2, slave_addr, I2C_Direction_Transmitter);    //发送从机地址，设置为写入
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
		
	I2C_SendData(I2C2, reg_addr);    //发送写入的地址
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	I2C_SendData(I2C2, data);         //发送写入的数据
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	I2C_GenerateSTOP(I2C2, ENABLE);      //结束
}

void i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char* data, int count)
{ 
	//static int count1 = 0;         //调试
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))         //等待空闲
	{
		//GPIO_Write(GPIOF, 0x0f0f);
	}

  I2C_GenerateSTART(I2C2, ENABLE);                             //发送开始信号
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));  //EV5

  I2C_Send7bitAddress(I2C2, slave_addr, I2C_Direction_Transmitter);            //从机地址,发送模式
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));   //EV6

  I2C_Cmd(I2C2, ENABLE);                                         //用重新使能的方式清空EV6
	
  I2C_SendData(I2C2, reg_addr);                                       //读取的地址
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));    //EV8

  I2C_GenerateSTART(I2C2, ENABLE);                               //再次发送开始信号
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));     //EV5

  I2C_Send7bitAddress(I2C2, slave_addr, I2C_Direction_Receiver);    //发送从机地址，接收模式
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));   //EV6
	


  while(count)
  {
    if(count == 1)
    {
      I2C_AcknowledgeConfig(I2C2, DISABLE);            //以无应答结束信号

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




//************************************************模拟IIC程序****************************************************************
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//先使能外设IO PORTC时钟 
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	IIC_SCL=1;
	IIC_SDA=1;

}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
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
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
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
	
	IIC_Send_Byte(slave_addr & 0xfe);   //从机地址
	
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
	
	IIC_Send_Byte(slave_addr & 0xfe);   //从机地址
	
	IIC_Wait_Ack();
	
	IIC_Send_Byte(reg_addr);
	
	IIC_Wait_Ack();
	
	IIC_Start();
	
	IIC_Send_Byte(slave_addr | 0x01);   //从机地址
	
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


//**********************************************移植的正点原子IIC函数********************************************************
void MPU_IIC_Delay(void)
{
	delay_us(2);
}


//产生IIC起始信号
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();     //sda线输出
	MPU_IIC_SDA=1;	  	  
	MPU_IIC_SCL=1;
	MPU_IIC_Delay();
 	MPU_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay();
	MPU_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();//sda线输出
	MPU_IIC_SCL=0;
	MPU_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay();
	MPU_IIC_SCL=1;  
	MPU_IIC_SDA=1;//发送I2C总线结束信号
	MPU_IIC_Delay();							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN();      //SDA设置为输入  
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
	MPU_IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void MPU_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL=0;//拉低时钟开始数据传输
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
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA设置为输入
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
        MPU_IIC_NAck();//发送nACK
    else
        MPU_IIC_Ack(); //发送ACK   
    return receive;
}
//***************************************************************************************************************************

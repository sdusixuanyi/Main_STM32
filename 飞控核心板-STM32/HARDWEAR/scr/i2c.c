#include "i2c.h"

void i2c_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	I2C_InitTypeDef  I2C_InitStructure; 

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

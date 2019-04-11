#include "Headfile.h"
#include "uart.h"
#include "Usart.h"
#include "Ringbuf.h"
//����ѭ�����л������ݶ���
RingBuff_t COM0_Rx_Buf,COM1_Rx_Buf,COM2_Rx_Buf,COM3_Rx_Buf,COM4_Rx_Buf,COM5_Rx_Buf,COM6_Rx_Buf,COM7_Rx_Buf;
void UART0_IRQHandler(void)//UART0�жϺ���
{	
  //��ȡ�жϱ�־ ԭʼ�ж�״̬ �������жϱ�־		
  uint32_t flag = UARTIntStatus(UART0_BASE,1);
  //����жϱ�־	
  UARTIntClear(UART0_BASE,flag);		
  //�ж�FIFO�Ƿ�������		
  while(UARTCharsAvail(UART0_BASE))		
  {			
    RingBuf_Write(UARTCharGet(UART0_BASE),&COM0_Rx_Buf,32);//�����ζ�������д����
    //RDroneStudio_Receive(UARTCharGet(UART0_BASE));		
  }
}


void ConfigureUART0(void)//����0��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);//ʹ��UART����
  GPIOPinConfigure(GPIO_PA0_U0RX);//GPIOģʽ���� PA0--RX PA1--TX 
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO��UARTģʽ����
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(0, 115200, 16000000);
  //UARTЭ������ ������115200 8λ 1ֹͣλ  ��У��λ	
  //UART����FIFO Ĭ��FIFO LevelΪ4/8 �Ĵ�����8�ֽں�����ж�	//���ú����1λ�Ͳ����ж�	
  UARTFIFODisable(UART0_BASE);//ʹ��UART0�ж�	IntEnable(INT_UART0);	
  UARTIntEnable(UART0_BASE,UART_INT_RX);//ʹ��UART0�����ж�		
  UARTIntRegister(UART0_BASE,UART0_IRQHandler);//UART�жϵ�ַע��	
  IntPrioritySet(INT_UART0, USER_INT2);
}


void USART0_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART0_BASE, *pui8Buffer++);
  }
}

void wust_sendware(unsigned char *wareaddr, int16_t waresize)//ɽ�ⷢ�Ͳ���
{
#define CMD_WARE     3
  uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};//֡ͷ
  uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};//֡β
  USART0_Send(cmdf, sizeof(cmdf));
  USART0_Send(wareaddr, waresize);
  USART0_Send(cmdr, sizeof(cmdr));
}


void UART1_IRQHandler(void)//UART1�жϺ���
{				
  uint32_t flag = UARTIntStatus(UART1_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־	
  UARTIntClear(UART1_BASE,flag);//����жϱ�־	
  while(UARTCharsAvail(UART1_BASE))//�ж�FIFO�Ƿ�������		
  {			
    //RingBuf_Write(UARTCharGet(UART1_BASE),&COM1_Rx_Buf,50);//�����ζ�������д����	
    ANO_DT_Data_Receive_Prepare(UARTCharGet(UART1_BASE)); 		
  }
}



void USART1_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART1_BASE, *pui8Buffer++);
  }
}

void ConfigureUART1(void)//����1��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);//ʹ��UART����
  GPIOPinConfigure(GPIO_PB0_U1RX);//GPIOģʽ���� PB0--RX PB1--TX 
  GPIOPinConfigure(GPIO_PB1_U1TX);
  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO��UARTģʽ����
  UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(1, 115200, 16000000);
  UARTFIFODisable(UART1_BASE);//ʹ��UART1�ж�	
  UARTIntEnable(UART1_BASE,UART_INT_RX);//ʹ��UART1�����ж�		
  UARTIntRegister(UART1_BASE,UART1_IRQHandler);//UART1�жϵ�ַע��	
  IntPrioritySet(INT_UART1, USER_INT4);
}






//UART2�жϺ���
void UART2_IRQHandler(void)
{
  uint32_t flag = UARTIntStatus(UART2_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־		
  UARTIntClear(UART2_BASE,flag);//����жϱ�־	
  while(UARTCharsAvail(UART2_BASE))//�ж�FIFO�Ƿ�������				
  {		
    //����õ�������
    RingBuf_Write(UARTCharGet(UART2_BASE),&COM2_Rx_Buf,200);//�����ζ�������д����
    if(COM2_Rx_Buf.Ring_Buff[0]!=0XB5)
    {
      COM2_Rx_Buf.Head=1;
      COM2_Rx_Buf.Tail=0; 
    }		
  }
}


void USART2_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART2_BASE, *pui8Buffer++);
  }
}

void ConfigureUART2(unsigned long bound)//����2��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);//ʹ��UART����
  
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;//����PD6
  HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;//ȷ��
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;//��������
  
  GPIOPinConfigure(GPIO_PD6_U2RX);//GPIOģʽ���� PD6--RX PD7--TX 
  GPIOPinConfigure(GPIO_PD7_U2TX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO��UARTģʽ����
  UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), bound,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART2_BASE);//ʹ��UART2�ж�	
  UARTIntEnable(UART2_BASE,UART_INT_RX |UART_INT_RT);//ʹ��UART0�����ж�		
  UARTIntRegister(UART2_BASE,UART2_IRQHandler);//UART�жϵ�ַע��	
  IntPrioritySet(INT_UART2, USER_INT1);
}



//UART2�жϺ���
void UART3_IRQHandler(void)
{		
  uint32_t flag = UARTIntStatus(UART3_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־		
  UARTIntClear(UART3_BASE,flag);//����жϱ�־			
  while(UARTCharsAvail(UART3_BASE))//�ж�FIFO�Ƿ�������		
  {			
    RingBuf_Write(UARTCharGet(UART3_BASE),&COM3_Rx_Buf,24);//�����ζ�������д����
    //SDK_Data_Receive_Prepare(UARTCharGet(UART3_BASE));		
  }
}


void USART3_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART3_BASE, *pui8Buffer++);
  }
}
void ConfigureUART3(void)//����3��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);//ʹ��UART����
  GPIOPinConfigure(GPIO_PC6_U3RX);//GPIOģʽ���� PC6--RX PC7--TX 
  GPIOPinConfigure(GPIO_PC7_U3TX);
  GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO��UARTģʽ����
  UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART3_BASE);//ʹ��UART0�ж�	
  UARTIntEnable(UART3_BASE,UART_INT_RX);//ʹ��UART3�����ж�		
  UARTIntRegister(UART3_BASE,UART3_IRQHandler);//UART3�жϵ�ַע��	
  IntPrioritySet(INT_UART3, USER_INT3);
}



//UART2�жϺ���
void UART6_IRQHandler(void)
{		
  uint32_t flag = UARTIntStatus(UART6_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־		
  UARTIntClear(UART6_BASE,flag);//����жϱ�־	
  while(UARTCharsAvail(UART6_BASE))//�ж�FIFO�Ƿ�������		
  {			
    RingBuf_Write(UARTCharGet(UART6_BASE),&COM6_Rx_Buf,28);//�����ζ�������д����		
  }
}


void USART6_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART6_BASE, *pui8Buffer++);
  }
}
void ConfigureUART6(void)//����6��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);//ʹ��UART����
  GPIOPinConfigure(GPIO_PD4_U6RX);//GPIOģʽ���� PD4--RX PD5--TX 
  GPIOPinConfigure(GPIO_PD5_U6TX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);//GPIO��UARTģʽ����
  UARTConfigSetExpClk(UART6_BASE, SysCtlClockGet(), 19200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART6_BASE);//ʹ��UART0�ж�	
  UARTIntEnable(UART6_BASE,UART_INT_RX);//ʹ��UART6�����ж�		
  OpticalFlow_Init();//�����˲�������ʼ��
  OpticalFlow_Is_Work=Config_Init_Uart();//������������ʼ��
  UARTIntRegister(UART6_BASE,UART6_IRQHandler);//UART6�жϵ�ַע��	
  IntPrioritySet(INT_UART6, USER_INT5);
}




void UART7_IRQHandler(void)//UART2�жϺ���
{		
  uint32_t flag = UARTIntStatus(UART7_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־	
  UARTIntClear(UART7_BASE,flag);//����жϱ�־		
  while(UARTCharsAvail(UART7_BASE))//�ж�FIFO�Ƿ�������			
  {			
    //����õ�������			
    //UARTCharPut(UART1_BASE,UARTCharGet(UART1_BASE));
    RingBuf_Write(UARTCharGet(UART7_BASE),&COM7_Rx_Buf,4);//�����ζ�������д����		
  }
}


void USART7_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART7_BASE, *pui8Buffer++);
  }
}
void ConfigureUART7(void)//����7��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);//ʹ��UART����
  GPIOPinConfigure(GPIO_PE0_U7RX);//GPIOģʽ���� PE0--RX PE1--TX 
  GPIOPinConfigure(GPIO_PE1_U7TX);
  GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO��UARTģʽ����
  UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), 9600,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART7_BASE);//ʹ��UART0�ж�	
  UARTIntEnable(UART7_BASE,UART_INT_RX);//ʹ��UART0�����ж�		
  UARTIntRegister(UART7_BASE,UART7_IRQHandler);//UART�жϵ�ַע��	
  IntPrioritySet(INT_UART7, USER_INT6);
}


/***********************************************************/
uint8_t data_to_send[50];//ANO����վ�������ݻ���
uint8_t ANO_Send_PID_Flag[6]={0};//PID���ͱ�־λ
#define vs16 int16_t
#define vs32 int32_t
#define s16  int16_t
/***********************************************************************/
void ANO_DT_Data_Receive_Prepare(u8 data)
{
    static u8 RxBuffer[50];
    static u8 _data_len = 0,_data_cnt = 0;
    static u8 state = 0;
    
    if(state==0&&data==0xAA)
    {
        state=1;
        RxBuffer[0]=data;
    }
    else if(state==1&&data==0xAF)
    {
        state=2;
        RxBuffer[1]=data;
    }else if (state==2&&data==0x05)
		{
			state=3;
			RxBuffer[2]=data;
		}
    else if(state==3&&data<0XF1)
    {
        state=4;
        RxBuffer[3]=data;
    }
    else if(state==4&&data<50)
    {
        state = 5;
        RxBuffer[4]=data;
        _data_len = data;
        _data_cnt = 0;
    }
    else if(state==5&&_data_len>0)
    {
        _data_len--;
        RxBuffer[5+_data_cnt++]=data;
        if(_data_len==0)
            state = 6;
    }
    else if(state==6)
    {
        state = 0;
        RxBuffer[5+_data_cnt]=data;
        ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+6);//У����ɣ����ݸ�ʽ����Э��Ҫ��
    }
    else
        state = 0;			//����У�飬�������Ķ���ķ��ͷ�ʽ�ƶ�����У��׼��
}


void ANO_DT_Check(u8 num)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0x05;
	data_to_send[2]=0xAF;
	data_to_send[3]=0xE1;
	data_to_send[4]=0x06;
	data_to_send[5]=0x00;
	data_to_send[6]=num;
	data_to_send[7]=0x00;
	data_to_send[8]=0x00;
	data_to_send[9]=0x00;
	data_to_send[10]=0x01;
	data_to_send[11]=0x46+num;
	USART1_Send(data_to_send,12);
}


void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)//ANO���ݽ���
{
    u8 sum = 0,i;
    for(i=0;i<(num-1);i++)
        sum += *(data_buf+i);
    if(!(sum==*(data_buf+num-1)))       return ;     //�ж�sum
    if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF && *(data_buf+2)==0x05))     return  ;     //�ж�֡ͷ
	
    if(*(data_buf+3)==0XE0)//У׼��������������Ʋ�ͬģ�����У׼
    {
			if (*(data_buf + 5) == 0x01)
			{
				switch (*(data_buf + 7))				
				{				
				case 0x01:  break;		//ACCУ׼
				case 0x02:  break;		//GYROУ׼
				case 0x04:  break;		//MAGУ׼
				case 0x05:  break;		//BAROУ׼
				default: break;
				}
			}
			else if(*(data_buf+5)==0x02)
			{
				switch (*(data_buf + 7))				
				{				
				case 0xAA:  Sort_PID_Flag=3; break;		//�ָ�Ĭ��PID
				case 0xAB:  break;		//�ָ�Ĭ�ϲ���
				case 0xAC:  break;		//�ָ�Ĭ�Ϲ���
				case 0xAF:  break;		//ȫ���ָ�Ĭ��
				default: break;
				}
			}
			else if (*(data_buf + 5) == 0x10)
			{
				switch (*(data_buf+7))
				{
				case 0x01:	break;			//һ�����
				case 0x02:	break;			//һ������
				case 0x03:	break;			//����
				case 0x04:	break;			//�½�
				case 0x05:	break;			//ǰ��
				case 0x06:	break;			//����
				case 0x07:	break;			//����
				case 0x08:	break;			//����
				case 0x09:	break;			//����
				case 0x0A:	break;			//����
				case 0xA0:	break;			//����ͣ��
				default: break;
				}
			}else if(*(data_buf+5)==0xE1)
			{
				ANO_DT_Check(*(data_buf+7));
			}
			USART1_Send (data_buf, num);
    }
		    if(*(data_buf+3)==0XE1)		//��ȡ��������ģ��
    {
			switch (*(data_buf + 6))
			{
			case 1:	Total_Controller.Roll_Gyro_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 2:	Total_Controller.Roll_Gyro_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 3:	Total_Controller.Roll_Gyro_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 4:	Total_Controller.Pitch_Gyro_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 5:	Total_Controller.Pitch_Gyro_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 6:	Total_Controller.Pitch_Gyro_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 7:	Total_Controller.Yaw_Gyro_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 8:	Total_Controller.Yaw_Gyro_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 9:	Total_Controller.Yaw_Gyro_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 10:Total_Controller.Roll_Angle_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 11:Total_Controller.Roll_Angle_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 12:Total_Controller.Roll_Angle_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 13:Total_Controller.Pitch_Angle_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 14:Total_Controller.Pitch_Angle_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 15:Total_Controller.Pitch_Angle_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 16:Total_Controller.Yaw_Angle_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 17:Total_Controller.Yaw_Angle_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 9)); break;
			case 18:Total_Controller.Yaw_Angle_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 19:Total_Controller.High_Speed_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 20:Total_Controller.High_Speed_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 21:Total_Controller.High_Speed_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 22:Total_Controller.High_Position_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 23:Total_Controller.High_Position_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 24:Total_Controller.High_Position_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 25:Total_Controller.Latitude_Speed_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));
					Total_Controller.Longitude_Speed_Control.Kp = Total_Controller.Latitude_Speed_Control.Kp; break;
			case 26:Total_Controller.Latitude_Speed_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));
					Total_Controller.Longitude_Speed_Control.Ki = Total_Controller.Latitude_Speed_Control.Ki; break;
			case 27:Total_Controller.Latitude_Speed_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));
					Total_Controller.Longitude_Speed_Control.Kd = Total_Controller.Latitude_Speed_Control.Kd; break;
			case 28:Total_Controller.Latitude_Position_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));
					Total_Controller.Longitude_Position_Control.Kp = Total_Controller.Latitude_Position_Control.Kp; break;
			case 29:Total_Controller.Latitude_Position_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));
					Total_Controller.Longitude_Position_Control.Ki = Total_Controller.Latitude_Position_Control.Ki; break;
			case 30:Total_Controller.Latitude_Position_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));
					Total_Controller.Longitude_Position_Control.Kd = Total_Controller.Latitude_Position_Control.Kd; break;
			case 31:Total_Controller.High_Acce_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 32:Total_Controller.High_Acce_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 33:Total_Controller.High_Acce_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 34:Total_Controller.Optical_Position_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 35:Total_Controller.Optical_Position_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 36:Total_Controller.Optical_Position_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 37:Total_Controller.Optical_Speed_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 38:Total_Controller.Optical_Speed_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 39:Total_Controller.Optical_Speed_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 40:Total_Controller.SDK_Roll_Position_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 41:Total_Controller.SDK_Roll_Position_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 42:Total_Controller.SDK_Roll_Position_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 43:Total_Controller.SDK_Pitch_Position_Control.Kp = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 44:Total_Controller.SDK_Pitch_Position_Control.Ki = 0.001*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 45:Total_Controller.SDK_Pitch_Position_Control.Kd = 0.01*((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			default:Sort_PID_Flag = 1;
					Bling_Set(&Light_1, 1000, 50, 0.5, 0, GPIO_PORTF_BASE, GPIO_PIN_1, 0);
					Bling_Set(&Light_2, 1000, 50, 0.5, 0, GPIO_PORTF_BASE, GPIO_PIN_2, 0);
					Bling_Set(&Light_3, 1000, 50, 0.5, 0, GPIO_PORTF_BASE, GPIO_PIN_3, 0); break;
			}
			*(data_buf+1)=0x05;
			*(data_buf+2)=0xAF;
			USART1_Send (data_buf,num);
    }
}



void ANO_Data_Send_Status(void)//���ͻ�����Ϣ����̬������״̬��
{
  u8 _cnt=0;
  vs16 _temp;
  vs32 _temp2;
  u8 sum = 0;
  u8 i;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x05;
  data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;
  data_to_send[_cnt++]=0;
  
  _temp = (int)(Roll*100);
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = (int)(Pitch*100);
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = (int)(-Yaw_Temp*100);
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  
  _temp2 = (vs32)(100*NamelessQuad.Position[_YAW]);//��λcm
  data_to_send[_cnt++]=BYTE3(_temp2);
  data_to_send[_cnt++]=BYTE2(_temp2);
  data_to_send[_cnt++]=BYTE1(_temp2);
  data_to_send[_cnt++]=BYTE0(_temp2);
  
  data_to_send[_cnt++]=0x01;//����ģʽ
  data_to_send[_cnt++]=Controler_State;//����0������1
  
  data_to_send[4] = _cnt-5;
  sum = 0;
  for(i=0;i<_cnt;i++)
    sum += data_to_send[i];
  data_to_send[_cnt++]=sum;
  USART1_Send(data_to_send, _cnt);
}

void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)//���ʹ�����ԭʼ������
{
  u8 _cnt=0;
  vs16 _temp;
  u8 sum = 0;
  u8 i=0;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
  data_to_send[_cnt++]=0xAF;
  data_to_send[_cnt++]=0x02;
  data_to_send[_cnt++]=0;
  
  _temp = a_x;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = a_y;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = a_z;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  
  _temp = g_x;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = g_y;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = g_z;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  
  _temp = m_x;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = m_y;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = m_z;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  
  data_to_send[4] = _cnt-5;
  
  sum = 0;
  for(i=0;i<_cnt;i++)
    sum += data_to_send[i];
  data_to_send[_cnt++] = sum;
  USART1_Send(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)//����ң����ͨ������
{
  u8 _cnt=0;
  u8 i=0;
  u8 sum = 0;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
  data_to_send[_cnt++]=0xAF;
  data_to_send[_cnt++]=0x03;
  data_to_send[_cnt++]=0;
  data_to_send[_cnt++]=BYTE1(thr);
  data_to_send[_cnt++]=BYTE0(thr);
  data_to_send[_cnt++]=BYTE1(yaw);
  data_to_send[_cnt++]=BYTE0(yaw);
  data_to_send[_cnt++]=BYTE1(rol);
  data_to_send[_cnt++]=BYTE0(rol);
  data_to_send[_cnt++]=BYTE1(pit);
  data_to_send[_cnt++]=BYTE0(pit);
  data_to_send[_cnt++]=BYTE1(aux1);
  data_to_send[_cnt++]=BYTE0(aux1);
  data_to_send[_cnt++]=BYTE1(aux2);
  data_to_send[_cnt++]=BYTE0(aux2);
  data_to_send[_cnt++]=BYTE1(aux3);
  data_to_send[_cnt++]=BYTE0(aux3);
  data_to_send[_cnt++]=BYTE1(aux4);
  data_to_send[_cnt++]=BYTE0(aux4);
  data_to_send[_cnt++]=BYTE1(aux5);
  data_to_send[_cnt++]=BYTE0(aux5);
  data_to_send[_cnt++]=BYTE1(aux6);
  data_to_send[_cnt++]=BYTE0(aux6);
  
  data_to_send[4] = _cnt-5;
  
  sum = 0;
  for(i=0;i<_cnt;i++)
    sum += data_to_send[i];
  
  data_to_send[_cnt++]=sum;
  USART1_Send(data_to_send, _cnt);
}

void ANO_DT_Send_GPSData(u8 Fixstate,u8 GPS_Num,u32 log,u32 lat,int16 gps_head)//����GPS������Ϣ
{
  u8 sum = 0;
  u8 _cnt=0;
  u8 i=0;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
  data_to_send[_cnt++]=0xAF;
  data_to_send[_cnt++]=0x04;
  data_to_send[_cnt++]=0;
  data_to_send[_cnt++]=Fixstate;
  data_to_send[_cnt++]=GPS_Num;
  
  data_to_send[_cnt++]=BYTE3(log);
  data_to_send[_cnt++]=BYTE2(log);
  data_to_send[_cnt++]=BYTE1(log);
  data_to_send[_cnt++]=BYTE0(log);
  
  data_to_send[_cnt++]=BYTE3(lat);
  data_to_send[_cnt++]=BYTE2(lat);
  data_to_send[_cnt++]=BYTE1(lat);
  data_to_send[_cnt++]=BYTE0(lat);
  
  data_to_send[_cnt++]=BYTE1(gps_head);
  data_to_send[_cnt++]=BYTE0(gps_head);
  
  data_to_send[4] = _cnt-5;
  
  sum = 0;
  for(i=0;i<_cnt;i++)
    sum += data_to_send[i];
  
  data_to_send[_cnt++]=sum;
  USART1_Send(data_to_send, _cnt);
}
void ANO_DT_Send_Power(u16 votage, u16 current)
{
    u8 _cnt=0;
    u16 temp;
    u8 sum = 0,i;    
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x05;
		data_to_send[_cnt++]=0xAF;
    data_to_send[_cnt++]=0x05;
    data_to_send[_cnt++]=0;
    
    temp = votage;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);
    temp = current;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);
    data_to_send[4] = _cnt-5;
    

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    
    data_to_send[_cnt++]=sum;
    
    USART1_Send(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
    u8 _cnt=0;
    u8 sum = 0,i;    
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x05;
		data_to_send[_cnt++]=0xAF;
    data_to_send[_cnt++]=0x06;
    data_to_send[_cnt++]=0;
    
    data_to_send[_cnt++]=BYTE1(m_1);
    data_to_send[_cnt++]=BYTE0(m_1);
    data_to_send[_cnt++]=BYTE1(m_2);
    data_to_send[_cnt++]=BYTE0(m_2);
    data_to_send[_cnt++]=BYTE1(m_3);
    data_to_send[_cnt++]=BYTE0(m_3);
    data_to_send[_cnt++]=BYTE1(m_4);
    data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
    data_to_send[_cnt++]=BYTE0(m_5);
    data_to_send[_cnt++]=BYTE1(m_6);
    data_to_send[_cnt++]=BYTE0(m_6);
    data_to_send[_cnt++]=BYTE1(m_7);
    data_to_send[_cnt++]=BYTE0(m_7);
    data_to_send[_cnt++]=BYTE1(m_8);
    data_to_send[_cnt++]=BYTE0(m_8);
    
    data_to_send[4] = _cnt-5;
    

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    
    data_to_send[_cnt++]=sum;
    
    USART1_Send(data_to_send, _cnt);
}
void ANO_DT_Send_Udata(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
    u8 _cnt=0;
		u8 sum = 0,i;
    vs16 _temp;
    
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x05;
		data_to_send[_cnt++]=0xAF;
    data_to_send[_cnt++]=0xF1;
    data_to_send[_cnt++]=0;
    
    _temp = a_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = a_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = a_z;    
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    
    _temp = g_x;    
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_y;    
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_z;    
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    
    _temp = m_x;    
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_y;    
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_z;    
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    
    data_to_send[4] = _cnt-5;
    

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    
    USART1_Send(data_to_send, _cnt);
}
void ANO_SEND_StateMachine(void)//��������ѭ������
{
	static int16_t ANO_Cnt=0;
	u16 Power_V; 
  ANO_Cnt++;
  if(ANO_Cnt%5==0)
  {
    ANO_Data_Send_Status();
  }
  if(ANO_Cnt%10==0)
  {
    ANO_DT_Send_Senser((int16_t)WP_Sensor.accel_raw.x,(int16_t)WP_Sensor.accel_raw.y,(int16_t)WP_Sensor.accel_raw.z,
                       (int16_t)WP_Sensor.gyro_raw.x,(int16_t)WP_Sensor.gyro_raw.y,(int16_t)WP_Sensor.gyro_raw.z,
                       (int16_t)WP_Sensor.mag_raw.x,(int16_t)WP_Sensor.mag_raw.y,(int16_t)WP_Sensor.mag_raw.z);
  }
  if(ANO_Cnt%15==0)
  {
		ANO_DT_Send_Udata(0,0,0,0,0,0,0,0,0);
//    ANO_DT_Send_RCData(PPM_Databuf[2],PPM_Databuf[3],
//                       PPM_Databuf[0],PPM_Databuf[1],
//                       PPM_Databuf[4],PPM_Databuf[5],
//                       PPM_Databuf[6],PPM_Databuf[7],0,0);
  }
	if(ANO_Cnt%30==0)
	{
		 ANO_DT_Send_MotoPWM(Motor_PWM_1,Motor_PWM_2,Motor_PWM_3,Motor_PWM_4,0,0,0,0);
	}
	if(ANO_Cnt%50==0)
	{
		Power_V=(u16)ADC_StartSample(0);
		ANO_DT_Send_Power(Power_V,20);
	}
  if(ANO_Cnt%150==0)//��ǰ��ֹ���Ͷ���
  {
//    ANO_DT_Send_GPSData(1,GPS_Sate_Num,Longitude_Origion,Latitude_Origion,10);
    ANO_Cnt=0;
  }
}







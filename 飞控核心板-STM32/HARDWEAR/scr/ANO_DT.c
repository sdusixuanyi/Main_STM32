#include "ANO_DT.h"
#include "mpu6050.h"
#include "flight_control.h"
#include "24l01.h"
#include "HCSR04.h"
#include "usart.h"
#include "stm32f10x_usart.h"
#include "kalman.h"
#include "stmflash.h"
#include "adc.h"
#include "debug_para.h"
/////////////////////////////////////////////////////////////////////////////////////
//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	

dt_flag_t f;            //��Ҫ�������ݵı�־
u8 data_to_send[50];    //�������ݻ���
extern float motor_duty[4];
extern struct PID_parameter PID_pitch ;  //��̬��PID
extern struct PID_parameter PID_roll;
extern struct PID_parameter PID_yaw;

extern struct PID_parameter PID_gyro_x;  //���ٶ�PID
extern struct PID_parameter PID_gyro_y;
extern struct PID_parameter PID_gyro_z;

extern float accel[3];

extern struct PID_parameter PID_height;  //�߶�PID

extern struct Kalman_data kalman_accel_x ;     //����ļ��ٶȿ������˲����ݣ�������Ϊlast_p, Q, R, output��
extern struct Kalman_data kalman_accel_y ;
extern struct Kalman_data kalman_accel_z ;

extern struct Attitude_int offset;     //������̬��ʱ��6050��õ�����
extern struct Attitude_float attitude;   //������ǰ����̬����

extern float Power_V;

extern struct debug_data debug;

/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange��������������ݷ������󣬱�����ʵ��ÿ5ms����һ�δ�������������λ�������ڴ˺�����ʵ��
//�˺���Ӧ���û�ÿ1ms����һ��
void ANO_DT_Data_Exchange(void)
{
    static u8 cnt = 0;
    static u8 senser_cnt    = 10;
    static u8 status_cnt    = 15;
    static u8 motopwm_cnt   = 20;
		static u8 udata_cnt			=	25;
    static u8 power_cnt     = 50;			//����ֵ����ȷ�����ݴ���Ƶ��
    
    if((cnt % senser_cnt) == (senser_cnt-1))
        f.send_senser = 1;  

    if((cnt % status_cnt) == (status_cnt-1))
        f.send_status = 1;  
    
    if((cnt % motopwm_cnt) == (motopwm_cnt-1))
        f.send_motopwm = 1; 
    
		if((cnt % udata_cnt) == (udata_cnt-1))
				f.udata =1;
    if((cnt % power_cnt) == (power_cnt-1))
        f.send_power = 1;					//�������ֵ����־λ��һ
    
    cnt++;

/////////////////////////////////////////////////////////////////////////////////////
    if(f.send_status)//����UAV��ǰ״̬����̬�ǡ��߶�
    {
        f.send_status = 0;
        ANO_DT_Send_Status(attitude.roll, attitude.pitch, attitude.yaw, attitude.height,2,1);
    }   
/////////////////////////////////////////////////////////////////////////////////////
    else if(f.send_senser)//���䴫��������6050�ʹ����Ƶ�����
    {
        f.send_senser = 0;
        ANO_DT_Send_Senser(debug.temp_duty_Kd*100, debug.temp_duty_Kp*100, debug.angle,attitude.gyro_x,attitude.gyro_y,attitude.gyro_z,accel[0],accel[1],accel[2]);
    }   
  else if(f.udata)
	{
			f.udata=0;
		ANO_DT_Send_Udata(0,0,0,0,0,0,0,0,0);
	}
/////////////////////////////////////////////////////////////////////////////////////  

    else if(f.send_motopwm)//������PWM�ź�
    {
        f.send_motopwm = 0;
        ANO_DT_Send_MotoPWM(motor_duty[0],motor_duty[1],motor_duty[2],motor_duty[3],motor_duty[0],motor_duty[1],motor_duty[2],motor_duty[3]);
    }   
/////////////////////////////////////////////////////////////////////////////////////
    else if(f.send_power)//�����ص���
    {
        f.send_power = 0;
				Power_V=Get_Adc(1)*330*4.49/4096;	//������ʱ�ĵ�ص������������ݴ���
				if(Power_V<10.5)PID_yaw.Kp=1;
        ANO_DT_Send_Power(Power_V,20);
    } 
}

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data������Э�������з������ݹ���ʹ�õ��ķ��ͺ���
//��ֲʱ���û�Ӧ��������Ӧ�õ����������ʹ�õ�ͨ�ŷ�ʽ��ʵ�ִ˺���
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
	u8 t;
	for(t=0;t<length;t++)							//����ͨѶ��ʽ���ʺ��ڵ��԰潫���ݴ��䵽��λ��
	{
		USART_SendData(USART2,*dataToSend);
		while( USART_GetFlagStatus(USART2,USART_FLAG_TC)!= SET);
		dataToSend++;
	}
}


/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare������Э��Ԥ����������Э��ĸ�ʽ�����յ������ݽ���һ�θ�ʽ�Խ�������ʽ��ȷ�Ļ��ٽ������ݽ���
//��ֲʱ���˺���Ӧ���û���������ʹ�õ�ͨ�ŷ�ʽ���е��ã����紮��ÿ�յ�һ�ֽ����ݣ�����ô˺���һ��
//�˺������������ϸ�ʽ������֡�󣬻����е������ݽ�������

//���պ������Խ��յ������ݽ������ݽ�����ȷ���Ƿ���Ϲ涨�����ݴ����ʽ
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
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl������Э�����ݽ������������������Ƿ���Э���ʽ��һ������֡���ú��������ȶ�Э�����ݽ���У��
//У��ͨ��������ݽ��н�����ʵ����Ӧ����
//�˺������Բ����û����е��ã��ɺ���Data_Receive_Prepare�Զ�����
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
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
			ANO_DT_Send_Data(data_buf, num);
    }
    
    if(*(data_buf+3)==0XE1)		//��ȡ��������ģ��
    {
			switch (*(data_buf + 6))
			{
			case 1 :PID_gyro_x.Kp = 0.00001 * ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 2 :PID_gyro_x.Ki = 0.00001 * ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 3 :PID_gyro_x.Kd = 0.00001 * ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10)); break;
			case 4 :PID_gyro_y.Kp = 0.00001 * ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 5 :PID_gyro_y.Ki = 0.00001 * ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 6 :PID_gyro_y.Kd = 0.00001 * ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 7 :PID_gyro_z.Kp = 0.001 * ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 8 :PID_gyro_z.Ki = 0.001 * ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 9 :PID_gyro_z.Kd = 0.001 * ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 10:PID_roll.Kp	  = 0.001 * ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 11:PID_roll.Ki   = 0.001 * ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 12:PID_roll.Kd   = 0.001 * ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 13:PID_pitch.Kp  = 0.01 *  ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 14:PID_pitch.Ki  = 0.01 *  ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 15:PID_pitch.Kd  = 0.01 *  ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 16:PID_yaw.Kp    =         ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 17:PID_yaw.Ki    =         ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 18:PID_yaw.Kd    =         ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 19:PID_height.Kp = 0.001*  ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 20:PID_height.Ki = 0.001*  ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			case 21:PID_height.Kd = 0.001*  ((vs16)(*(data_buf + 9) << 8) | *(data_buf + 10));break;
			default:  break;
			}
			*(data_buf+1)=0x05;
			*(data_buf+2)=0xAF;
			ANO_DT_Send_Data(data_buf,num);
    }
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
	ANO_DT_Send_Data(data_to_send,12);
}
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
    u8 _cnt=0;
	u8 sum = 0,i;
    vs16 _temp;
    vs32 _temp2 = alt;
    
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0xAF;
    data_to_send[_cnt++]=0x01;
    data_to_send[_cnt++]=0;
    
    _temp = (int)(angle_rol*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(angle_pit*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(angle_yaw*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);		//����Ƕȶ�Ӧֵ
    
    data_to_send[_cnt++]=BYTE3(_temp2);
    data_to_send[_cnt++]=BYTE2(_temp2);
    data_to_send[_cnt++]=BYTE1(_temp2);
    data_to_send[_cnt++]=BYTE0(_temp2);		//����ALT�߶���Ϣ
    
    data_to_send[_cnt++] = fly_model;		//����˿̷���ģʽ
											//��	00
											//��̬	01
											//����	02
											//����	03
											//����	11
											//����	20
											//����	21

    data_to_send[_cnt++] = armed;			//����armed  0:������1������
    
    data_to_send[4] = _cnt-5;
    

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++]=sum;
    
    ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
    u8 _cnt=0;
	u8 sum = 0,i;
    vs16 _temp;
    
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
    

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    
    ANO_DT_Send_Data(data_to_send, _cnt);
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
    
    ANO_DT_Send_Data(data_to_send, _cnt);
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
    
    ANO_DT_Send_Data(data_to_send, _cnt);
}

//�����û�����ʹ��
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
    
    ANO_DT_Send_Data(data_to_send, _cnt);
}


#include "ANO_DT.h"
#include "mpu6050.h"
#include "flight_control.h"
#include "24l01.h"
#include "HCSR04.h"
#include "usart.h"
#include "stm32f10x_usart.h"
/////////////////////////////////////////////////////////////////////////////////////
//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)      ) )
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

extern struct PID_parameter PID_height;  //�߶�PID

extern struct Attitude_int offset;     //������̬��ʱ��6050��õ�����
extern struct Attitude_float attitude;   //������ǰ����̬����

extern float Power_V;

/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange��������������ݷ������󣬱�����ʵ��ÿ5ms����һ�δ�������������λ�������ڴ˺�����ʵ��
//�˺���Ӧ���û�ÿ1ms����һ��
void ANO_DT_Data_Exchange(void)
{
    static u8 cnt = 0;
    static u8 senser_cnt    = 10;
    static u8 status_cnt    = 15;
    static u8 motopwm_cnt   = 20;
    static u8 power_cnt     = 50;			//����ֵ����ȷ�����ݴ���Ƶ��
    
    if((cnt % senser_cnt) == (senser_cnt-1))
        f.send_senser = 1;  

    if((cnt % status_cnt) == (status_cnt-1))
        f.send_status = 1;  
    
    if((cnt % motopwm_cnt) == (motopwm_cnt-1))
        f.send_motopwm = 1; 
    
    if((cnt % power_cnt) == (power_cnt-1))
        f.send_power = 1;					//�������ֵ����־λ��һ
    
    cnt++;

/////////////////////////////////////////////////////////////////////////////////////
    if(f.send_status)//����UAV��ǰ״̬����̬�ǡ��߶�
    {
        f.send_status = 0;
        ANO_DT_Send_Status(attitude.roll, attitude.pitch, attitude.yaw, attitude.height,0,0);
    }   
/////////////////////////////////////////////////////////////////////////////////////
    else if(f.send_senser)//���䴫��������6050�ʹ�����
    {
        f.send_senser = 0;
        ANO_DT_Send_Senser(attitude.accel_x, attitude.accel_y, attitude.accel_z,attitude.gyro_x,attitude.gyro_y,attitude.gyro_z,0,0,0,0);
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
        ANO_DT_Send_Power(Power_V,20);
    }
/////////////////////////////////////////////////////////////////////////////////////
    else if(f.send_pid1)//����PID����
    {
        f.send_pid1 = 0;
        ANO_DT_Send_PID(1, PID_pitch.Kp, PID_pitch.Ki, PID_pitch.Kd, PID_roll.Kp, PID_roll.Ki, PID_roll.Kd, PID_yaw.Kp, PID_yaw.Ki, PID_yaw.Kd);
    }   
/////////////////////////////////////////////////////////////////////////////////////
    else if(f.send_pid2)
    {
        f.send_pid2 = 0;
		ANO_DT_Send_PID(2, PID_gyro_x.Kp, PID_gyro_x.Ki, PID_gyro_x.Kd, PID_gyro_y.Kp, PID_gyro_y.Ki, PID_gyro_y.Kd, PID_gyro_z.Kp, PID_gyro_z.Ki, PID_gyro_z.Kd);
	}
/////////////////////////////////////////////////////////////////////////////////////
    else if(f.send_pid3)
    {
        f.send_pid3 = 0;
		ANO_DT_Send_PID(3, PID_height.Kp, PID_height.Ki, PID_height.Kd, PID_height.Kp, PID_height.Ki, PID_height.Kd, PID_height.Kp, PID_height.Ki, PID_height.Kd);
	}           
/////////////////////////////////////////////////////////////////////////////////////
}

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data������Э�������з������ݹ���ʹ�õ��ķ��ͺ���
//��ֲʱ���û�Ӧ��������Ӧ�õ����������ʹ�õ�ͨ�ŷ�ʽ��ʵ�ִ˺���
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
		NRF24L01_TX_Mode();
		while(NRF24L01_TxPacket(dataToSend)==TX_OK);			//NRF���䷽ʽ�������ڷɿغ�����������ݴ���
//			u8 t;
//			for(t=0;t<length;t++)							//����ͨѶ��ʽ���ʺ��ڵ��԰潫���ݴ��䵽��λ��
//			{
//				USART_SendData(USART1,*dataToSend);
//				while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET); 	
//				dataToSend++;
//			}
}

static void ANO_DT_Send_Check(u8 head, u8 check_sum)//����У��ֵ
{
	  u8 sum = 0,i;
    data_to_send[0]=0xAA;
    data_to_send[1]=0xAA;
    data_to_send[2]=0xEF;
    data_to_send[3]=2;
    data_to_send[4]=head;
    data_to_send[5]=check_sum;
    for(i=0;i<6;i++)
        sum += data_to_send[i];
    data_to_send[6]=sum;
    ANO_DT_Send_Data(data_to_send, 7);
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
    }
    else if(state==2&&data<0XF1)
    {
        state=3;
        RxBuffer[2]=data;
    }
    else if(state==3&&data<50)
    {
        state = 4;
        RxBuffer[3]=data;
        _data_len = data;
        _data_cnt = 0;
    }
    else if(state==4&&_data_len>0)
    {
        _data_len--;
        RxBuffer[4+_data_cnt++]=data;
        if(_data_len==0)
            state = 5;
    }
    else if(state==5)
    {
        state = 0;
        RxBuffer[4+_data_cnt]=data;
        ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);//У����ɣ����ݸ�ʽ����Э��Ҫ��
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
    if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))     return  ;     //�ж�֡ͷ
    
    if(*(data_buf+2)==0X01)//У׼��������������Ʋ�ͬģ�����У׼
    {
//        if(*(data_buf+4)==0X01)
//            mpu6050.Acc_CALIBRATE = 1;
//        if(*(data_buf+4)==0X02)
//            mpu6050.Gyro_CALIBRATE = 1;
//        if(*(data_buf+4)==0X03)
//        {
//            mpu6050.Acc_CALIBRATE = 1;      
//            mpu6050.Gyro_CALIBRATE = 1;         
//        }
    }
    
    if(*(data_buf+2)==0X02)//��ȡ��������ģ��
    {
        if(*(data_buf+4)==0X01)//����PID���ģ����Ϣ
        {
            f.send_pid1 = 1;
            f.send_pid2 = 1;
            f.send_pid3 = 1;
//            f.send_pid4 = 1;
//            f.send_pid5 = 1;
//            f.send_pid6 = 1;
        }
        if(*(data_buf+4)==0X02)//���·������õ���Ϣ����ʱû���ô�
        {
            
        }
    }

    if(*(data_buf+2)==0X10)     //����PID1�����Ϣ
    {
        PID_roll.Kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        PID_roll.Ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        PID_roll.Kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        PID_pitch.Kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        PID_pitch.Ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        PID_pitch.Kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        PID_yaw.Kp   = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        PID_yaw.Ki   = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        PID_yaw.Kd   = 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
//                Param_SavePID();//���õ������ݸ��µ�PID����
    }
    if(*(data_buf+2)==0X11)     //����PID����
    {
        PID_gyro_x.Kp     = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        PID_gyro_x.Ki     = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        PID_gyro_x.Kd     = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        PID_gyro_y.Kp     = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        PID_gyro_y.Ki     = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        PID_gyro_y.Kd     = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        PID_gyro_z.Kp   	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        PID_gyro_z.Ki     = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        PID_gyro_z.Kd     = 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
//                Param_SavePID();
    }
    if(*(data_buf+2)==0X12)                             //PID3
    {   
        PID_height.Kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        PID_height.Ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        PID_height.Kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
//        ctrl_2.PID[PIDPITCH].kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//        ctrl_2.PID[PIDPITCH].ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//        ctrl_2.PID[PIDPITCH].kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//        ctrl_2.PID[PIDYAW].kp   = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        ctrl_2.PID[PIDYAW].ki   = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        ctrl_2.PID[PIDYAW].kd   = 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
//                Param_SavePID();
    }
    if(*(data_buf+2)==0X13)                             //PID4
    {
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
    if(*(data_buf+2)==0X14)                             //PID5
    {
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
    if(*(data_buf+2)==0X15)                             //PID6
    {
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
}


void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
    u8 _cnt=0;
	  u8 sum = 0,i;
    vs16 _temp;
    vs32 _temp2 = alt;
    
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
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
    data_to_send[_cnt++]=BYTE0(_temp2);		//����alt��
    
    data_to_send[_cnt++] = fly_model;		//����˿̷���ģʽ
    
    data_to_send[_cnt++] = armed;			//����armed
    
    data_to_send[3] = _cnt-4;
    

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++]=sum;
    
    ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar)
{
    u8 _cnt=0;
	  u8 sum = 0,i;
    vs16 _temp;
    
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
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
    
    data_to_send[3] = _cnt-4;
    

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
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x05;
    data_to_send[_cnt++]=0;
    
    temp = votage;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);
    temp = current;
    data_to_send[_cnt++]=BYTE1(temp);
    data_to_send[_cnt++]=BYTE0(temp);
    data_to_send[3] = _cnt-4;
    

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
    data_to_send[_cnt++]=0xAA;
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
    
    data_to_send[3] = _cnt-4;
    

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    
    data_to_send[_cnt++]=sum;
    
    ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
    u8 _cnt=0,i;
	  u8 sum = 0;
    vs16 _temp;
    
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x10+group-1;
    data_to_send[_cnt++]=0;
    
    
    _temp = p1_p * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p1_i  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p1_d  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p2_p  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p2_i  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p2_d * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p3_p  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p3_i  * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = p3_d * 1000;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    
    data_to_send[3] = _cnt-4;
    

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    
    data_to_send[_cnt++]=sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}



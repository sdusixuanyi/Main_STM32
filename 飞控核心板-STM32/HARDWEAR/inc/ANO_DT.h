#ifndef _ANO_DT_H
#define	_ANO_DT_H

#include "sys.h"

typedef struct 
{
		//u8 send_version;
		u8 send_status;
		u8 send_senser;
		u8 udata;
		u8 send_pid1;
		u8 send_pid2;
		u8 send_pid3;
		u8 send_pid4;
		u8 send_pid5;
		u8 send_pid6;
		u8 send_offset;
		u8 send_motopwm;
		u8 send_power;

}dt_flag_t;

extern dt_flag_t f;		//定义标志位结构体，存储各个参数的标志位

void ANO_DT_Data_Exchange(void);			//核心数据交换函数，每1ms调用一次（最快），外部只需要调用该函数即可
void ANO_DT_Data_Receive_Prepare(u8 data);
//内部调用函数，外部不需要调用
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num);
void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void ANO_DT_Send_Power(u16 votage, u16 current);
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void ANO_DT_Send_Udata(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void ANO_DT_Check(u8 num);

#endif


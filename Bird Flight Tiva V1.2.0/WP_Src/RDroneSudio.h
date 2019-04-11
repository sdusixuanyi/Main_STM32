#ifndef __RDRONESTUDIO_H__
#define __RDRONESTUDIO_H__
//float ÁªºÏÌå
typedef union{
        float fv;
        uint16_t sv[2];
}float_union;


void RDroneStudio_Send_Attitude(float rd_pitch,float rd_roll,float rd_yaw);
void RDroneStudio_Send_Gyro(float rd_pitch_gyro,float rd_roll_gyro,float rd_yaw_gyro);	
void RDroneStudio_Send_AcceData(float x,float y,float z);
void RDroneStudio_Send_MagData(float x,float y,float z);
void RDroneStudio_Send_Pos(float pos_z,float pos_x,float pos_y);
void RDroneStudio_Send_PID_ALL(void);

void RDroneStudio_Send(void);
void RDroneStudio_Receive(u8 data);
#endif



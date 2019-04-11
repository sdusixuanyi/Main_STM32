#include "Headfile.h"
#include "RDroneSudio.h"

/* 浮点数到十六进制转换 */
/* floatNum：浮点型变量 */
/* byteArry：分解成 4 个字节的首地址 */
void FloatToByte(float floatNum,unsigned char* byteArry)
{
	int i;
	char* pchar=(char*)&floatNum;
	for(i=0;i<sizeof(float);i++)
	{
		*byteArry=*pchar;
		pchar++;
		byteArry++;
	}
}

void RDroneStudio_Send_Attitude(float rd_pitch,float rd_roll,float rd_yaw)
{
  unsigned char rd_buf[16];
	rd_buf[0]=0x55;
	rd_buf[1]=0xAA;
  rd_buf[2]=0x00;
  FloatToByte(rd_pitch,&rd_buf[3]);
  FloatToByte(rd_roll,&rd_buf[7]);
  FloatToByte(rd_yaw,&rd_buf[11]);
	rd_buf[15]=0x55;
	USART0_Send(rd_buf, 16);
}
void RDroneStudio_Send_Pos(float pos_z,float pos_x,float pos_y)
{
  unsigned char rd_buf[16];
	rd_buf[0]=0x55;
	rd_buf[1]=0xAA;
  rd_buf[2]=0x01;
  FloatToByte(pos_z,&rd_buf[3]);
  FloatToByte(pos_x,&rd_buf[7]);
  FloatToByte(pos_y,&rd_buf[11]);
	rd_buf[15]=0x55;
	USART0_Send(rd_buf, 16);
}
void RDroneStudio_Send_Gyro(float rd_pitch_gyro,float rd_roll_gyro,float rd_yaw_gyro)
{
  unsigned char rd_buf[16];
	rd_buf[0]=0x55;
	rd_buf[1]=0xAA;
  rd_buf[2]=0x20;
  FloatToByte(rd_pitch_gyro,&rd_buf[3]);
  FloatToByte(rd_roll_gyro,&rd_buf[7]);
  FloatToByte(rd_yaw_gyro,&rd_buf[11]);
	rd_buf[15]=0x00;
	USART0_Send(rd_buf, 16);
}

void RDroneStudio_Send_AcceData(float x,float y,float z)
{
  unsigned char rd_buf[16];
	rd_buf[0]=0x55;
	rd_buf[1]=0xAA;
  rd_buf[2]=0x21;
  FloatToByte(x,&rd_buf[3]);
  FloatToByte(y,&rd_buf[7]);
  FloatToByte(z,&rd_buf[11]);
	rd_buf[15]=0x00;
	USART0_Send(rd_buf, 16);
}

void RDroneStudio_Send_MagData(float x,float y,float z)
{
  unsigned char rd_buf[16];
	rd_buf[0]=0x55;
	rd_buf[1]=0xAA;
  rd_buf[2]=0x22;
  FloatToByte(x,&rd_buf[3]);
  FloatToByte(y,&rd_buf[7]);
  FloatToByte(z,&rd_buf[11]);
	rd_buf[15]=0x00;
	USART0_Send(rd_buf, 16);
}

void RDroneStudio_Send_PID(unsigned char label,float kp,float ki,float kd)
{
  unsigned char rd_buf[16];
	rd_buf[0]=0x55;
	rd_buf[1]=0xAA;
  rd_buf[2]=label;
  FloatToByte(kp,&rd_buf[3]);
  FloatToByte(ki,&rd_buf[7]);
  FloatToByte(kd,&rd_buf[11]);
	rd_buf[15]=0x00;
	USART0_Send(rd_buf, 16);
}

void RDroneStudio_Send_PID_ALL(void)
{
	RDroneStudio_Send_PID(0x02,Total_Controller.Pitch_Angle_Control.Kp
	                          ,Total_Controller.Pitch_Angle_Control.Ki
                            ,Total_Controller.Pitch_Angle_Control.Kd);
 	RDroneStudio_Send_PID(0x03,Total_Controller.Roll_Angle_Control.Kp
	                          ,Total_Controller.Roll_Angle_Control.Ki
                            ,Total_Controller.Roll_Angle_Control.Kd);
	RDroneStudio_Send_PID(0x04,Total_Controller.Pitch_Gyro_Control.Kp
	                          ,Total_Controller.Pitch_Gyro_Control.Ki
                            ,Total_Controller.Pitch_Gyro_Control.Kd);
 	RDroneStudio_Send_PID(0x05,Total_Controller.Roll_Gyro_Control.Kp
	                          ,Total_Controller.Roll_Gyro_Control.Ki
                            ,Total_Controller.Roll_Gyro_Control.Kd);	
  RDroneStudio_Send_PID(0x06,Total_Controller.Yaw_Angle_Control.Kp
	                          ,Total_Controller.Yaw_Angle_Control.Ki
                            ,Total_Controller.Yaw_Angle_Control.Kd);
  RDroneStudio_Send_PID(0x07,Total_Controller.Yaw_Gyro_Control.Kp
	                          ,Total_Controller.Yaw_Gyro_Control.Ki
                            ,Total_Controller.Yaw_Gyro_Control.Kd);
	
	RDroneStudio_Send_PID(0x08,Total_Controller.High_Position_Control.Kp
	                          ,Total_Controller.High_Position_Control.Ki
                            ,Total_Controller.High_Position_Control.Kd);
  RDroneStudio_Send_PID(0x09,Total_Controller.High_Speed_Control.Kp
	                          ,Total_Controller.High_Speed_Control.Ki
                            ,Total_Controller.High_Speed_Control.Kd);
	
  RDroneStudio_Send_PID(0x0A,Total_Controller.Latitude_Position_Control.Kp
	                          ,Total_Controller.Latitude_Position_Control.Ki
                            ,Total_Controller.Latitude_Position_Control.Kd);
  RDroneStudio_Send_PID(0x0B,Total_Controller.Latitude_Speed_Control.Kp
	                          ,Total_Controller.Latitude_Speed_Control.Ki
                            ,Total_Controller.Latitude_Speed_Control.Kd);
	RDroneStudio_Send_PID(0x0C,Total_Controller.Longitude_Position_Control.Kp
	                          ,Total_Controller.Longitude_Position_Control.Ki
                            ,Total_Controller.Longitude_Position_Control.Kd);
  RDroneStudio_Send_PID(0x0D,Total_Controller.Longitude_Speed_Control.Kp
	                          ,Total_Controller.Longitude_Speed_Control.Ki
                            ,Total_Controller.Longitude_Speed_Control.Kd);													
	
}  

u8 RDroneStudio_RxBuffer[50];	
void RDroneStudio_Data_Anl(unsigned char *PCData);
void RDroneStudio_Receive(u8 data)
{	
	static u8 _data_cnt = 0;
	static u8 state = 0;
	if(state==0&&data==0x55)
	{
		state=1;
		RDroneStudio_RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAA)
	{
		state=2;
		RDroneStudio_RxBuffer[1]=data;
	}
	else if(state==2)
	{
		state=3;
		RDroneStudio_RxBuffer[2]=data;
	}
	else if(state==3&&_data_cnt<12)
	{
		RDroneStudio_RxBuffer[_data_cnt++]=data;
		state = 4;
	}
	else if(state==4)
	{
		state = 0;
		RDroneStudio_RxBuffer[15]=data;
		//RDroneStudio_Data_Anl(RDroneStudio_RxBuffer);
	}
	else state = 0;
}



uint8_t RDroneStudio_Send_PID_FLAG=0;
void RDroneStudio_Send(void)
{
	RDroneStudio_Send_Attitude(Pitch,Roll,Yaw);
	RDroneStudio_Send_Gyro(Pitch_Gyro,Roll_Gyro,Yaw_Gyro);
	RDroneStudio_Send_AcceData(accel.x/AcceMax,accel.y/AcceMax,accel.z/AcceMax);
	RDroneStudio_Send_MagData(mag.x,mag.y,mag.z);
	RDroneStudio_Send_Pos(NamelessQuad.Position[_YAW]/100.0f,NamelessQuad.Position[_PITCH]/100.0f,NamelessQuad.Position[_ROLL]/100.0f);
	if(RDroneStudio_Send_PID_FLAG==1)
	{
		RDroneStudio_Send_PID_ALL();
		RDroneStudio_Send_PID_FLAG=0;
	}
}

float_union HexToFloat;
float UnsignedcharToFloat(unsigned char *DataofPC,unsigned char sequence)
{
    HexToFloat.sv[0] = (((uint16_t)DataofPC[sequence+1]) << 8 | DataofPC[sequence]);
    HexToFloat.sv[1] = (((uint16_t)DataofPC[sequence+3]) << 8 | DataofPC[sequence+2]);
    return HexToFloat.fv;
}

void RDroneStudio_Data_Anl(unsigned char *PCData)
{
        /*读取PID*/
        if(PCData[2]==0x03)
        {
           RDroneStudio_Send_PID_FLAG=1;
        }
        /* 设置Pitch的PID参数 */
        else if(PCData[2]==0x07)
				{
            Total_Controller.Pitch_Angle_Control.Kp= UnsignedcharToFloat(PCData,3);
            Total_Controller.Pitch_Angle_Control.Ki = UnsignedcharToFloat(PCData,7);
            Total_Controller.Pitch_Angle_Control.Kd = UnsignedcharToFloat(PCData,11);
            RDroneStudio_Send_PID_FLAG=1;
            //Write_Config();
        }
        /* 设置ROll的PID参数 */
        else if(PCData[2]==0x08)
				{
            Total_Controller.Roll_Angle_Control.Kp = UnsignedcharToFloat(PCData,3);
            Total_Controller.Roll_Angle_Control.Ki = UnsignedcharToFloat(PCData,7);
            Total_Controller.Roll_Angle_Control.Kd = UnsignedcharToFloat(PCData,11);
            RDroneStudio_Send_PID_FLAG=1;
            //Write_Config();
        }
        /* 设置Yaw的PID参数 */
        else if(PCData[2]==0x09)
				{
            Total_Controller.Yaw_Angle_Control.Kp = UnsignedcharToFloat(PCData,3);
            Total_Controller.Yaw_Angle_Control.Ki = UnsignedcharToFloat(PCData,7);
            Total_Controller.Yaw_Angle_Control.Kd = UnsignedcharToFloat(PCData,11);
            RDroneStudio_Send_PID_FLAG=1;
            //Write_Config();
        }
        /* 设置Height的PID参数 */
        else if(PCData[2]==0x0A)
				{
            Total_Controller.High_Position_Control.Kp = UnsignedcharToFloat(PCData,3);
            Total_Controller.High_Position_Control.Ki = UnsignedcharToFloat(PCData,7);
            Total_Controller.High_Position_Control.Kd = UnsignedcharToFloat(PCData,11);
            RDroneStudio_Send_PID_FLAG=1;
            //Write_Config();
        }
        /* 设置accPitch的PID参数 */
        else if(PCData[2]==0x0B)
				{
            Total_Controller.Pitch_Gyro_Control.Kp = UnsignedcharToFloat(PCData,3);
            Total_Controller.Pitch_Gyro_Control.Ki = UnsignedcharToFloat(PCData,7);
            Total_Controller.Pitch_Gyro_Control.Kd = UnsignedcharToFloat(PCData,11);
            RDroneStudio_Send_PID_FLAG=1;
            //Write_Config();
        }
        /* 设置accRoll的PID参数 */
        else if(PCData[2]==0x0C)
				{
            Total_Controller.Roll_Gyro_Control.Kp = UnsignedcharToFloat(PCData,3);
            Total_Controller.Roll_Gyro_Control.Ki = UnsignedcharToFloat(PCData,7);
            Total_Controller.Roll_Gyro_Control.Kd = UnsignedcharToFloat(PCData,11);
            RDroneStudio_Send_PID_FLAG=1;
            //Write_Config();
        }
        /* 设置accYaw的PID参数 */
        else if(PCData[2]==0x0D)
				{
            Total_Controller.Yaw_Gyro_Control.Kp = UnsignedcharToFloat(PCData,3);
            Total_Controller.Yaw_Gyro_Control.Ki = UnsignedcharToFloat(PCData,7);
            Total_Controller.Yaw_Gyro_Control.Kd = UnsignedcharToFloat(PCData,11);
            RDroneStudio_Send_PID_FLAG=1;
            //Write_Config();
        }
        /* 设置accHeight的PID参数 */
        else if(PCData[2]==0x0E)
				{
            Total_Controller.High_Speed_Control.Kp = UnsignedcharToFloat(PCData,3);
            Total_Controller.High_Speed_Control.Ki = UnsignedcharToFloat(PCData,7);
            Total_Controller.High_Speed_Control.Kd = UnsignedcharToFloat(PCData,11);
            RDroneStudio_Send_PID_FLAG=1;
            //Write_Config();
        }
        /* 设置PositionX的PID参数 */
        else if(PCData[2]==0x12)
				{
            Total_Controller.Latitude_Position_Control.Kp = UnsignedcharToFloat(PCData,3);
            Total_Controller.Latitude_Position_Control.Ki = UnsignedcharToFloat(PCData,7);
            Total_Controller.Latitude_Position_Control.Kd = UnsignedcharToFloat(PCData,11);
            RDroneStudio_Send_PID_FLAG=1;
            //Write_Config();
        }
        /* 设置PositionY的PID参数 */
        else if(PCData[2]==0x13)
				{
            Total_Controller.Longitude_Position_Control.Kp = UnsignedcharToFloat(PCData,3);
            Total_Controller.Longitude_Position_Control.Ki = UnsignedcharToFloat(PCData,7);
            Total_Controller.Longitude_Position_Control.Kd = UnsignedcharToFloat(PCData,11);
            RDroneStudio_Send_PID_FLAG=1;
            //Write_Config();
        }
        /* 设置PositionVX的PID参数 */
        else if(PCData[2]==0x14)
				{
            Total_Controller.Latitude_Speed_Control.Kp = UnsignedcharToFloat(PCData,3);
            Total_Controller.Latitude_Speed_Control.Ki = UnsignedcharToFloat(PCData,7);
            Total_Controller.Latitude_Speed_Control.Kd = UnsignedcharToFloat(PCData,11);
            RDroneStudio_Send_PID_FLAG=1;
            //Write_Config();
        }
        /* 设置PositionVY的PID参数 */
        else if(PCData[2]==0x15)
				{
            Total_Controller.Longitude_Speed_Control.Kp = UnsignedcharToFloat(PCData,3);
            Total_Controller.Longitude_Speed_Control.Ki = UnsignedcharToFloat(PCData,7);
            Total_Controller.Longitude_Speed_Control.Kd = UnsignedcharToFloat(PCData,11);
            RDroneStudio_Send_PID_FLAG=1;
            //Write_Config();
        }
        /*陀螺仪校准*/
        else if(PCData[2]==0x30)
        {
            //OffsetData.GyroX  = RT_Info.GyroX ;
            //OffsetData.GyroY  = RT_Info.GyroY ;
           // OffsetData.GyroZ  = RT_Info.GyroZ ;
           // Write_Config();
        }
}


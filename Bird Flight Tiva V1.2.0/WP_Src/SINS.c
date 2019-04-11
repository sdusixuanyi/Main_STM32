#include "Headfile.h"
#include "Earth_Declination.h"
#include "SINS.h"
void Vector_From_BodyFrame2EarthFrame(Vector3f *bf,Vector3f *ef)
{
  ef->x=rMat[0][0]*bf->x+rMat[0][1]*bf->y+rMat[0][2]*bf->z;
  ef->y=rMat[1][0]*bf->x+rMat[1][1]*bf->y+rMat[1][2]*bf->z;
  ef->z=rMat[2][0]*bf->x+rMat[2][1]*bf->y+rMat[2][2]*bf->z;
}

void Vector_From_EarthFrame2BodyFrame(Vector3f *ef,Vector3f *bf)
{
  bf->x=rMat[0][0]*ef->x+rMat[1][0]*ef->y+rMat[2][0]*ef->z;
  bf->y=rMat[0][1]*ef->x+rMat[1][1]*ef->y+rMat[2][1]*ef->z;
  bf->z=rMat[0][2]*ef->x+rMat[1][2]*ef->y+rMat[2][2]*ef->z;
}

SINS NamelessQuad;
SINS Origion_NamelessQuad;
Butter_BufferData accel_feedback_filter_buf[3];
float Acceleration_Feedback[3];
Vector2f SINS_Accel_Body={0,0};
void  SINS_Prepare(void)
{
  Vector2f SINS_Accel_Earth={0,0};
  Vector3f Body_Frame,Earth_Frame;
  /*Z-Y-X欧拉角转方向余弦矩阵
  //Pitch Roll  Yaw 分别对应Φ θ Ψ  
  X轴旋转矩阵
  R（Φ）
  {1      0        0    }
  {0      cosΦ    sinΦ}
  {0    -sinΦ    cosΦ }
  
  Y轴旋转矩阵
  R（θ）
  {cosθ     0        -sinθ}
  {0         1        0     }
  {sinθ     0        cosθ}
  
  Z轴旋转矩阵
  R（θ）
  {cosΨ      sinΨ       0}
  {-sinΨ     cosΨ       0}
  {0          0           1 }
  
  由Z-Y-X顺规有:
  载体坐标系到导航坐标系下旋转矩阵R(b2n)
  R(b2n) =R(Ψ)^T*R(θ)^T*R(Φ)^T
  
  R=
  {cosΨ*cosθ     -cosΦ*sinΨ+sinΦ*sinθ*cosΨ        sinΨ*sinΦ+cosΦ*sinθ*cosΨ}
  {cosθ*sinΨ     cosΦ*cosΨ +sinΦ*sinθ*sinΨ       -cosΨ*sinΦ+cosΦ*sinθ*sinΨ}
  {-sinθ          cosθsin Φ                          cosθcosΦ                   }
  */
  Body_Frame.x=ins_accel_filter.x;
  Body_Frame.y=ins_accel_filter.y;
  Body_Frame.z=ins_accel_filter.z;
  Vector_From_BodyFrame2EarthFrame(&Body_Frame,&Earth_Frame);
  Origion_NamelessQuad.Acceleration[_YAW]=Earth_Frame.z;
  Origion_NamelessQuad.Acceleration[_PITCH]=Earth_Frame.x;
  Origion_NamelessQuad.Acceleration[_ROLL]=Earth_Frame.y;
  
  
  Origion_NamelessQuad.Acceleration[_YAW]*=AcceGravity/AcceMax;
  Origion_NamelessQuad.Acceleration[_YAW]-=AcceGravity;//减去重力加速度
  Origion_NamelessQuad.Acceleration[_YAW]*=100;//加速度cm/s^2
  
  Origion_NamelessQuad.Acceleration[_PITCH]*=AcceGravity/AcceMax;
  Origion_NamelessQuad.Acceleration[_PITCH]*=100;//加速度cm/s^2
  
  Origion_NamelessQuad.Acceleration[_ROLL]*=AcceGravity/AcceMax;
  Origion_NamelessQuad.Acceleration[_ROLL]*=100;//加速度cm/s^2
  
  /******************************************************************************/
  //将无人机在导航坐标系下的沿着正东、正北方向的运动加速度旋转到当前航向的运动加速度:机头(俯仰)+横滚
  
  SINS_Accel_Earth.x=Origion_NamelessQuad.Acceleration[_PITCH];//沿地理坐标系，正东方向运动加速度,单位为CM
  SINS_Accel_Earth.y=Origion_NamelessQuad.Acceleration[_ROLL];//沿地理坐标系，正北方向运动加速度,单位为CM
  
  SINS_Accel_Body.x=SINS_Accel_Earth.x*Cos_Yaw+SINS_Accel_Earth.y*Sin_Yaw;  //横滚正向运动加速度  X轴正向
  SINS_Accel_Body.y=-SINS_Accel_Earth.x*Sin_Yaw+SINS_Accel_Earth.y*Cos_Yaw; //机头正向运动加速度  Y轴正向
  
  
  Body_Frame.x=LPButterworth(accel.x,&accel_feedback_filter_buf[0],&Butter_5HZ_Parameter_Acce);
  Body_Frame.y=LPButterworth(accel.y,&accel_feedback_filter_buf[1],&Butter_5HZ_Parameter_Acce);;
  Body_Frame.z=LPButterworth(accel.z,&accel_feedback_filter_buf[2],&Butter_5HZ_Parameter_Acce);;
  Vector_From_BodyFrame2EarthFrame(&Body_Frame,&Earth_Frame);
  Acceleration_Feedback[_YAW]=Earth_Frame.z;
  Acceleration_Feedback[_PITCH]=Earth_Frame.x;
  Acceleration_Feedback[_ROLL]=Earth_Frame.y;
  Acceleration_Feedback[_YAW]*=AcceGravity/AcceMax;
  Acceleration_Feedback[_YAW]-=AcceGravity;//减去重力加速度
  Acceleration_Feedback[_YAW]*=100;//加速度cm/s^2
  Acceleration_Feedback[_PITCH]*=AcceGravity/AcceMax;
  Acceleration_Feedback[_PITCH]*=100;//加速度cm/s^2
  Acceleration_Feedback[_ROLL]*=AcceGravity/AcceMax;
  Acceleration_Feedback[_ROLL]*=100;//加速度cm/s^2
}






#define KALMAN_DT 0.005f
float R[2]={5.0e-4f,6.0e-4f};//float R[2]={5.0e-4f,3.0e-4f};//{5.0e-4f,3.0e-4f}
float Q=15;//50
float Acce_Bias_Gain[3]={
  0.0005,//0.001
  0.0005,//0.001
  0.0005,//0.001
};
float Pre_conv[4]=
{
  0.18,0.1,//0.001,0,
  0.1,0.18//0,0.001
};//上一次协方差
void  KalmanFilter(float Observation,//位置观测量
                   uint16 Pos_Delay_Cnt,//观测传感器延时量
                   SINS *Ins_Kf,//惯导结构体
                   float System_drive,//系统原始驱动量，惯导加速度
                   float *R,
                   float Q,
                   float dt,
                   uint16 N,
                   uint8_t update_flag)
{
  uint16 Cnt=0;
  static uint16 Speed_Sync_Cnt=0;
  float Temp_conv[4]={0};//先验协方差
  float Conv_Z=0,Z_Cor=0;
  float k[2]={0};//增益矩阵
  float Ctemp=0;
  //先验状态
  Ins_Kf->Acceleration[N]=System_drive;
  Ins_Kf->Acceleration[N]=Ins_Kf->Acce_Bias[N]+Ins_Kf->Acceleration[N];
  Ins_Kf->Position[N] +=Ins_Kf->Speed[N]*dt+(Ins_Kf->Acceleration[N]*dt*dt)/2.0f;
  Ins_Kf->Speed[N]+=Ins_Kf->Acceleration[N]*dt;
  if(update_flag==1)
  {
    //先验协方差
    Ctemp=Pre_conv[1]+Pre_conv[3]*dt;
    Temp_conv[0]=Pre_conv[0]+Pre_conv[2]*dt+Ctemp*dt+R[0];
    Temp_conv[1]=Ctemp;
    Temp_conv[2]=Pre_conv[2]+Pre_conv[3]*dt;;
    Temp_conv[3]=Pre_conv[3]+R[1];
    //计算卡尔曼增益
    Conv_Z=Temp_conv[0]+Q;
    k[0]=Temp_conv[0]/Conv_Z;
    k[1]=Temp_conv[2]/Conv_Z;
    //融合数据输出
    Z_Cor=Observation-Ins_Kf->Pos_History[N][Pos_Delay_Cnt];
    //Z_Cor=Positional-*Position;
    Ins_Kf->Position[N] +=k[0]*Z_Cor;
    Ins_Kf->Speed[N] +=k[1]*Z_Cor;
    Ins_Kf->Acce_Bias[N]+=Acce_Bias_Gain[_YAW]*Z_Cor;
    //更新状态协方差矩阵
    Pre_conv[0]=(1-k[0])*Temp_conv[0];
    Pre_conv[1]=(1-k[0])*Temp_conv[1];
    Pre_conv[2]=Temp_conv[2]-k[1]*Temp_conv[0];
    Pre_conv[3]=Temp_conv[3]-k[1]*Temp_conv[1]; 
  }
  
  for(Cnt=Num-1;Cnt>0;Cnt--)//2.5ms滑动一次
  {
    Ins_Kf->Pos_History[N][Cnt]=Ins_Kf->Pos_History[N][Cnt-1];
  }
  Ins_Kf->Pos_History[N][0]=Ins_Kf->Position[N];
  
  Speed_Sync_Cnt++;
  if(Speed_Sync_Cnt>=40)//100ms滑动一次
  {
    for(Cnt=Num-1;Cnt>0;Cnt--)
    {
      Ins_Kf->Vel_History[N][Cnt]=Ins_Kf->Vel_History[N][Cnt-1];
    }
    Speed_Sync_Cnt=0;
  }
  Ins_Kf->Vel_History[N][0]=Ins_Kf->Speed[N];
}



#define SPL06_Sync_Cnt 40
#define HCSR04_Sync_Cnt 40
uint16 High_Delay_Cnt=0;
uint8_t Observation_Update_Flag=0;
float Observation_Altitude=0;
void Observation_Tradeoff(uint8_t HC_SR04_Enable)
{
  static uint8_t observation_flag=1,last_observation_flag=1;
  uint16 Cnt=0;
  last_observation_flag=observation_flag;
  if(HC_SR04_Enable==0)//无超声波参与
  {
    Observation_Altitude=WP_Sensor.baro_altitude;//高度观测量
    High_Delay_Cnt=SPL06_Sync_Cnt;
    Observation_Update_Flag=1;
    observation_flag=1;
  }
  else//有超声波参与
  {    
    if(Sensor_Flag.Hcsr04_Health==1)
    {
      Observation_Altitude=US_Distance;
      High_Delay_Cnt=HCSR04_Sync_Cnt;
      observation_flag=2;
      if(US_100_Update_Flag==1)
      {
        Observation_Update_Flag=US_100_Update_Flag;
        US_100_Update_Flag=0;
      }
    }
    else
    {
      Observation_Altitude=WP_Sensor.baro_altitude;//高度观测量
      High_Delay_Cnt=SPL06_Sync_Cnt;
      Observation_Update_Flag=1;
      observation_flag=1;
    } 
    
    if(observation_flag==2&&last_observation_flag==1)//气压计切超声波
    {
      NamelessQuad.Position[_YAW]=US_Distance;
      for(Cnt=Num-1;Cnt>0;Cnt--){NamelessQuad.Pos_History[_YAW][Cnt]=US_Distance;}
      NamelessQuad.Pos_History[_YAW][0]=US_Distance;
      Total_Controller.High_Position_Control.Expect=US_Distance;//将惯导高度设置为期望高度，有且仅设置一次
      
      Origion_NamelessQuad.Position[_YAW]=US_Distance;
      Origion_NamelessQuad.Speed[_YAW]=0;
    }
    else if(observation_flag==1&&last_observation_flag==2)//超声波切气压计
    {
      NamelessQuad.Position[_YAW]=Observation_Altitude;
      for(Cnt=Num-1;Cnt>0;Cnt--){NamelessQuad.Pos_History[_YAW][Cnt]=Observation_Altitude;}
      NamelessQuad.Pos_History[_YAW][0]=Observation_Altitude;
      Total_Controller.High_Position_Control.Expect=Observation_Altitude;//将惯导高度设置为期望高度，有且仅设置一次
      Origion_NamelessQuad.Position[_YAW]=Observation_Altitude;
      Origion_NamelessQuad.Speed[_YAW]=0;
    }
  }
}

Testime SINS_High_Delta;
float Delta_T;
void Strapdown_INS_High_Kalman(void)
{
  Test_Period(&SINS_High_Delta);
  Delta_T=SINS_High_Delta.Time_Delta/1000.0f;
  if(Delta_T>2*WP_Duty_Dt||Delta_T<WP_Duty_Dt||isnan(Delta_T)!=0)   Delta_T=WP_Duty_Dt;
  Observation_Tradeoff(1);
  KalmanFilter(Observation_Altitude,//位置观测量
               High_Delay_Cnt,//观测传感器延时量
               &NamelessQuad,//惯导结构体
               Origion_NamelessQuad.Acceleration[_YAW],//系统原始驱动量，惯导加速度
               R,
               Q,
               Delta_T,
               _YAW,
               1);
}

/*************************以下计算球面投影距离内容源于APM3.2 AP.Math.c文件******************************/
float longitude_scale(Location loc)
{
  static int32_t last_lat;
  static float scale = 1.0;
  //比较两次纬度相差值，避免重复运算余弦函数
  if (ABS(last_lat - loc.lat) < 100000) {
    // we are within 0.01 degrees (about 1km) of the
    // same latitude. We can avoid the cos() and return
    // the same scale factor.
    return scale;
  }
  scale = cosf(loc.lat * 1.0e-7f * DEG_TO_RAD);
  scale = constrain_float(scale, 0.01f, 1.0f);
  last_lat = loc.lat;
  return scale;
}
/*
return the distance in meters in North/East plane as a N/E vector
from loc1 to loc2
*/
Vector2f location_diff(Location loc1,Location loc2)
{
  Vector2f Location_Delta;
  Location_Delta.x=(loc2.lat - loc1.lat) * LOCATION_SCALING_FACTOR;//距离单位为m
  Location_Delta.y=(loc2.lng - loc1.lng) * LOCATION_SCALING_FACTOR * longitude_scale(loc2);//距离单位为m
  return Location_Delta;
}

// return distance in meters between two locations
float get_distance(Location loc1,Location loc2){
  float dlat              = (float)(loc2.lat - loc1.lat);
  float dlong             = ((float)(loc2.lng - loc1.lng)) * longitude_scale(loc2);
  return pythagorous2(dlat, dlong) * LOCATION_SCALING_FACTOR;
}
/*************************以上计算球面投影距离内容源于APM3.2 AP.Math.c文件******************************/
float Body_Frame_To_XYZ[3]={0,0,0};//导航（地理）坐标系，机体横滚(Y正)、俯仰(X正)方向位置偏移
Vector3_Nav Earth_Frame_To_XYZ={0,0,0};//导航（地理）坐标系，天 、正北、正东方向上位置偏移
Vector2i GPSData_To_XY_Home={1143637460,306276000};//经、纬度
Location GPS_Home;//初始定位成功点信息
Location GPS_Present;//当前位置点信息
uint8_t GPS_Home_Set=0;
float Declination=0;//地球磁偏角
void Set_GPS_Home(void)//设置Home点
{
  if(GPS_Home_Set==0
     &&GPS_Sate_Num>=8)//全程只设置一次
  {
    GPSData_To_XY_Home.x=Longitude_Origion;//Longitude;
    GPSData_To_XY_Home.y=Latitude_Origion;//Latitude;
    GPS_Home_Set=1;//设定返航点
    GPS_Home.lng=GPSData_To_XY_Home.x;
    GPS_Home.lat=GPSData_To_XY_Home.y; 
    Strapdown_INS_Reset(&NamelessQuad,_PITCH,Earth_Frame_To_XYZ.E,0);//复位惯导融合
    Strapdown_INS_Reset(&NamelessQuad,_ROLL,Earth_Frame_To_XYZ.N,0);//复位惯导融合
    Declination=get_declination(0.0000001f*GPS_Home.lat,0.0000001f*GPS_Home.lng);//获取当地磁偏角
  }
}


void GPSData_Sort()
{
  Vector2f location_delta={0,0};
  GPS_Present.lng=Longitude_Origion;//更新当前经纬度
  GPS_Present.lat=Latitude_Origion;
  location_delta=location_diff(GPS_Home,GPS_Present);//根据当前GPS定位信息与Home点位置信息计算正北、正东方向位置偏移
  /***********************************************************************************
  明确下导航系方向，这里正北、正东为正方向:
  沿着正东，经度增加,当无人机相对home点，往正东向移动时，此时GPS_Present.lng>GPS_Home.lng，得到的location_delta.x大于0;
  沿着正北，纬度增加,当无人机相对home点，往正北向移动时，此时GPS_Present.lat>GPS_Home.lat，得到的location_delta.y大于0;
  ******************************************************************************/
  location_delta.x*=100;//沿地理坐标系，正北方向位置偏移,单位为CM
  location_delta.y*=100;//沿地理坐标系，正东方向位置偏移,单位为CM
  Earth_Frame_To_XYZ.E=location_delta.y;//地理系下相对Home点正东位置偏移，单位为CM
  Earth_Frame_To_XYZ.N=location_delta.x;//地理系下相对Home点正北位置偏移，单位为CM
  //将无人机在导航坐标系下的沿着正东、正北方向的位置偏移旋转到当前航向的位置偏移:机头(俯仰)+横滚
  Body_Frame_To_XYZ[_PITCH]=location_delta.x*Cos_Yaw+location_delta.y*Sin_Yaw;//机头正向位置偏移  Y轴正向
  Body_Frame_To_XYZ[_ROLL]=-location_delta.x*Sin_Yaw+location_delta.y*Cos_Yaw;  //横滚正向位置偏移  X轴正向
}



#define Dealt_Update 0.1 //100ms更新一次
#define Process_Noise_Constant 1.0
#define Pos_Process_Noise  0.5*Process_Noise_Constant*Dealt_Update*Dealt_Update//0.005
#define Vel_Process_Noise  Process_Noise_Constant*Dealt_Update  //0.1
float R_GPS[2]={Pos_Process_Noise,Vel_Process_Noise};
float Q_GPS[2]={0.075,1.2};//{0.075,1.2};
float R_Acce_bias[2]={0.0001,0};
double Pre_conv_GPS[2][4]=
{
  0.018 ,    0.005,  0.005    , 0.5,//0.0001 ,    0.00001,  0.00001    , 0.003,
  0.018 ,    0.005,  0.005    , 0.5,
};//上一次协方差
double K_GPS[2][2]={0};//增益矩阵
float Acce_Bias[2]={0,0.001};
void   KalmanFilter_Horizontal_GPS(float Position_GPS,float Vel_GPS,float Position_Last,float Vel_Last,
                                   float *Position,float *Vel,
                                   float *Acce,float *R,
                                   float *Q,float dt,uint8_t Axis)
{
  float Conv_Z=0;
  float Z_Delta[2]={0};
  float Conv_Temp=0;
  double Temp_conv[4]={0};//先验协方差
  uint8 Label=0;
  if(Axis=='X') Label=0;
  else Label=1;
  //先验状态
  *Position +=*Vel*dt+((*Acce+Acce_Bias[Label])*dt*dt)/2.0f;
  *Vel+=(*Acce+Acce_Bias[Label])*dt;
  //先验协方差
  Conv_Temp=Pre_conv_GPS[Label][1]+Pre_conv_GPS[Label][3]*dt;
  Temp_conv[0]=Pre_conv_GPS[Label][0]+Pre_conv_GPS[Label][2]*dt+Conv_Temp*dt+R_GPS[0];
  Temp_conv[1]=Conv_Temp;
  Temp_conv[2]=Pre_conv_GPS[Label][2]+Pre_conv_GPS[Label][3]*dt;
  Temp_conv[3]=Pre_conv_GPS[Label][3]+R_GPS[1];
  //计算卡尔曼增益
  Conv_Z=1.0/((Temp_conv[0]+Q_GPS[0]*GPS_Quality)*(Temp_conv[3]+Q_GPS[1]*GPS_Quality)-Temp_conv[1]*Temp_conv[2]);
  //化简如下
  K_GPS[0][0]=( Temp_conv[0]*(Temp_conv[3]+Q_GPS[1]*GPS_Quality)-Temp_conv[1]*Temp_conv[2])*Conv_Z;
  K_GPS[0][1]=(Temp_conv[1]*Q_GPS[0]*GPS_Quality)*Conv_Z;
  K_GPS[1][0]=(Temp_conv[2]*Q_GPS[1]*GPS_Quality)*Conv_Z;
  K_GPS[1][1]=(-Temp_conv[1]*Temp_conv[2]+Temp_conv[3]*(Temp_conv[0]+Q_GPS[0]*GPS_Quality))*Conv_Z;
  //融合数据输出
  Z_Delta[0]=Position_GPS-Position_Last;
  Z_Delta[1]=Vel_GPS-Vel_Last;
  *Position +=K_GPS[0][0]*Z_Delta[0]
    +K_GPS[0][1]*Z_Delta[1];
  *Vel +=K_GPS[1][0]*Z_Delta[0]
    +K_GPS[1][1]*Z_Delta[1];
  Acce_Bias[Label]+=R_Acce_bias[0]*Z_Delta[0]
    +R_Acce_bias[1]*Z_Delta[1];
  //更新状态协方差矩阵
  Pre_conv_GPS[Label][0]=(1-K_GPS[0][0])*Temp_conv[0]-K_GPS[0][1]*Temp_conv[2];
  Pre_conv_GPS[Label][1]=(1-K_GPS[0][0])*Temp_conv[1]-K_GPS[0][1]*Temp_conv[3];
  Pre_conv_GPS[Label][2]=(1-K_GPS[1][1])*Temp_conv[2]-K_GPS[1][0]*Temp_conv[0];
  Pre_conv_GPS[Label][3]=(1-K_GPS[1][1])*Temp_conv[3]-K_GPS[1][0]*Temp_conv[1];
}

void Strapdown_INS_Reset(SINS *Ins,uint8_t Axis,float Pos_Target,float Vel_Target)
{
  uint16_t Cnt=0;
  Ins->Position[Axis]=Pos_Target;//位置重置
  Ins->Speed[Axis]=Vel_Target;//速度重置
  Ins->Acceleration[Axis]=0.0f;//加速度清零
  Ins->Acce_Bias[Axis]=0.0f;
  for(Cnt=Num-1;Cnt>0;Cnt--)//历史位置值，全部装载为当前观测值
  {
    Ins->Pos_History[Axis][Cnt]=Pos_Target;
  }
  Ins->Pos_History[Axis][0]=Pos_Target;
  for(Cnt=Num-1;Cnt>0;Cnt--)//历史速度值，全部装载为当前观测值
  {
    Ins->Vel_History[Axis][Cnt]=Vel_Target;
  }
  Ins->Vel_History[Axis][0]=Vel_Target;
}


#define X_Axis  0
#define Y_Axis  1
#define NUM_BUF 50
float Position_History[2][NUM_BUF]={0};
float Vel_History[2][NUM_BUF]={0};
uint16 GPS_Vel_Delay_Cnt=20;// 200ms 20-200ms   5
uint16 GPS_Pos_Delay_Cnt=10;//100ms  20-200ms   10
int16 GPS_Position_Cnt=0;
float Acce_bias[2]={0};
Testime SINS_Horizontal_Delta;
float Horizontal_Delta_T=0;
uint8_t Filter_Defeated_Flag=0;
void KalmanFilter_Horizontal(void)
{
  int16 i=0;
  if(GPS_Home_Set==1)
  {
    GPSData_Sort();
    Test_Period(&SINS_Horizontal_Delta);
    Horizontal_Delta_T=(float)(SINS_Horizontal_Delta.Time_Delta/1000.0f);
    if(Horizontal_Delta_T>2*WP_Duty_Dt||Horizontal_Delta_T<WP_Duty_Dt||isnan(Horizontal_Delta_T)!=0)   Horizontal_Delta_T=WP_Duty_Dt;
    GPS_Position_Cnt++;
    if(GPS_Position_Cnt>=2)//每10ms保存一次
    {
      for(i=NUM_BUF-1;i>0;i--)
      {
        Position_History[X_Axis][i]=Position_History[X_Axis][i-1];
        Position_History[Y_Axis][i]=Position_History[Y_Axis][i-1];
        Vel_History[X_Axis][i]=Vel_History[X_Axis][i-1];
        Vel_History[Y_Axis][i]=Vel_History[Y_Axis][i-1];
      }
      Position_History[X_Axis][0]=NamelessQuad.Position[_PITCH];
      Position_History[Y_Axis][0]=NamelessQuad.Position[_ROLL];
      Vel_History[X_Axis][0]=NamelessQuad.Speed[_PITCH];
      Vel_History[Y_Axis][0]=NamelessQuad.Speed[_ROLL];
      GPS_Position_Cnt=0;
    }
    NamelessQuad.Acceleration[_PITCH]=Origion_NamelessQuad.Acceleration[_PITCH];
    NamelessQuad.Acceleration[_ROLL]=Origion_NamelessQuad.Acceleration[_ROLL];
    if(GPS_Update_Flag==1)
    {
      KalmanFilter_Horizontal_GPS(Earth_Frame_To_XYZ.E,
                                  GPS_Speed_Resolve[1],
                                  Position_History[X_Axis][GPS_Pos_Delay_Cnt],
                                  Vel_History[X_Axis][GPS_Vel_Delay_Cnt],
                                  &NamelessQuad.Position[_PITCH],
                                  &NamelessQuad.Speed[_PITCH],
                                  &Origion_NamelessQuad.Acceleration[_PITCH],
                                  R_GPS,Q_GPS,Horizontal_Delta_T,'X');
      KalmanFilter_Horizontal_GPS(Earth_Frame_To_XYZ.N,
                                  GPS_Speed_Resolve[0],
                                  Position_History[Y_Axis][GPS_Pos_Delay_Cnt],
                                  Vel_History[Y_Axis][GPS_Vel_Delay_Cnt],
                                  &NamelessQuad.Position[_ROLL],
                                  &NamelessQuad.Speed[_ROLL],
                                  &Origion_NamelessQuad.Acceleration[_ROLL],
                                  R_GPS,Q_GPS,Horizontal_Delta_T,'Y');
      GPS_Update_Flag=0;
    }
    else
    {
      NamelessQuad.Position[_PITCH] +=NamelessQuad.Speed[_PITCH]*Horizontal_Delta_T
        +((NamelessQuad.Acceleration[_PITCH])*Horizontal_Delta_T*Horizontal_Delta_T)/2.0f;
      NamelessQuad.Speed[_PITCH]+=((NamelessQuad.Acceleration[_PITCH]))*Horizontal_Delta_T;
      
      NamelessQuad.Position[_ROLL] +=NamelessQuad.Speed[_ROLL]*Horizontal_Delta_T
        +((NamelessQuad.Acceleration[_ROLL])*Horizontal_Delta_T*Horizontal_Delta_T)/2.0f;
      NamelessQuad.Speed[_ROLL]+=(NamelessQuad.Acceleration[_ROLL])*Horizontal_Delta_T;
    }
    
    if(ABS(NamelessQuad.Position[_PITCH]-Earth_Frame_To_XYZ.E)>10000
       ||ABS(NamelessQuad.Position[_ROLL]-Earth_Frame_To_XYZ.N)>10000
         ||ABS(NamelessQuad.Speed[_PITCH]-GPS_Speed_Resolve[1])>10000
           ||ABS(NamelessQuad.Speed[_ROLL]-GPS_Speed_Resolve[0])>10000
             )
    {
      Filter_Defeated_Flag=1;//开始时，融合失败标志
    }
  }
}

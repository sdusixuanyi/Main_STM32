#ifndef __SENSOR_H
#define __SENSOR_H

extern float K[3];
extern float B[3];
extern volatile float Sin_Pitch,Sin_Roll,Sin_Yaw;
extern volatile float Cos_Pitch,Cos_Roll,Cos_Yaw;

#define RtA         57.324841
#define AtR    	    0.0174533
#define Acc_G 	    0.0000610351
#define Gyro_G 	    0.0610351
#define Gyro_Gr	    0.0010653
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)
//#define GYRO_CALIBRATION_COFF 0.060976f  //2000
#define GYRO_CALIBRATION_COFF 0.030488f  //1000	
#define AcceMax_1G      4096
#define GRAVITY_MSS     9.80665f
#define ACCEL_TO_1G     GRAVITY_MSS/AcceMax_1G
#define One_G_TO_Accel  AcceMax_1G/GRAVITY_MSS
void WP_Quad_Init(void);
void Senosr_Update(void);	
void Observation_Angle_Calculate(void);
void Madgwick_AHRS_Update_IMU(float gx, float gy, float gz, float ax, float ay, float az,float gyro_mold) ;
void Get_Status_Feedback(void);
extern volatile float Pitch,Roll,Yaw,Yaw_Gyro,Pitch_Gyro,Roll_Gyro;
extern float Yaw_Gyro_Earth_Frame;
extern float Pitch_Observation,Roll_Observation,Yaw_Observation,Altitude_Observation;
extern volatile float rMat[3][3];
extern Sensor WP_Sensor;
extern Vector3f gyro,accel,mag;
extern Vector3f gyro_filter,accel_filter,mag_filter,ins_accel_filter;
extern Sensor_Health Sensor_Flag;
extern Vector3f_Body Circle_Angle;
extern float Gyro_Length;
extern float q0,q1,q2,q3;
extern volatile float IMU_Dt;
extern float Yaw_Temp; 
extern float Accel_For_Cal[3];


extern uint16_t Bug_Cnt[2];
#endif


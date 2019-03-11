#ifndef _FLIGHT_H_
#define _FLIGHT_H_

#include "kalman.h"
#include "mpu6050.h"
#include "myMath.h"

#define COMMON_READ 0
#define DMA_READ    1

#define ERR_STORE_NUM 10    //���洢��������
#define STA_DUTY 0          //���ռ�ձȳ�ʼֵ
#define DUTY_MAX 1          //����ʱ��������ռ�ձ�
#define DUTY_MIN 0.3        //����ʱ�������Сռ�ձ�

#define DEBUG_MODE_GYRO_X 0    //���Գ����µ�ģʽѡ��
#define DEBUG_MODE_GYRO_Y 1
#define DEBUG_MODE_GYRO_Z 2

#define DEBUG_MODE_ROLL   3
#define DEBUG_MODE_PITCH  4
#define DEBUG_MODE_YAW    5

#define DEBUG_MODE_HEIGHT 6

#define LIM_ROLL_MAX   80            //Ŀ��Ƕ��޷�����limit()��������  Ĭ��0��Ϊ������
#define LIM_ROLL_MIN  -80
#define LIM_PITCH_MAX  80
#define LIM_PITCH_MIN -80
#define LIM_YAW_MAX    0
#define LIM_YAW_MIN    0

// #define LIM_ERR_ROLL_MAX 0        //�Ƕ�����޷�
// #define LIM_ERR_ROLL_MIN 0
// #define LIM_ERR_PITCH_MAX 0
// #define LIM_ERR_PITCH_MIN 0
// #define LIM_ERR_YAW_MAX 0
// #define LIM_ERR_YAW_MIN 0

#define LIM_GYRO_X_MAX 90         //Ŀ����ٶ��޷�
#define LIM_GYRO_X_MIN -90
#define LIM_GYRO_Y_MAX 90
#define LIM_GYRO_Y_MIN -90
#define LIM_GYRO_Z_MAX 90
#define LIM_GYRO_Z_MIN -90

#define LIM_HEIGHT_MAX 0       //Ŀ��߶��޷�
#define LIM_HEIGHT_MIN 0

#define LIM_DELTA_DUTY_ROLL_MAX 0     //����͸߶ȸ��Ե�PID�����޷�
#define LIM_DELTA_DUTY_ROLL_MIN 0
#define LIM_DELTA_DUTY_PITCH_MAX 0
#define LIM_DELTA_DUTY_PITCH_MIN 0
#define LIM_DELTA_DUTY_YAW_MAX 0
#define LIM_DELTA_DUTY_YAW_MIN 0
#define LIM_DELTA_DUTY_HEIGHT_MAX 0
#define LIM_DELTA_DUTY_HEIGHT_MIN 0

// #define LIM_TEMP_DELTA_DUTY_MAX 0       //�ܵ�ռ�ձ������޷�   �������ã���Ϊ�����а�������������ά�ȵ���Ϣ���������޷��������̬������Ϣ���ƻ�
// #define LIM_TEMP_DELTA_DUTY_MIN 0


struct Attitude_float
{
	float accel_x;
	float accel_y;
	float accel_z;
	
	float gyro_x;
	float gyro_y;
	float gyro_z;
	
	float pitch;
	float roll;
	float yaw;
	
	float height;
};

struct Attitude_int
{
	short int accel_x;
	short int accel_y;
	short int accel_z;
	
	short int gyro_x;
	short int gyro_y;
	short int gyro_z;
	
	float pitch;
	float roll;
	float yaw;
};

struct Vector_t
{
	float x;
	float y;
	float z;
};

struct Quaternion
{
	float q0;
	float q1;
	float q2;
	float q3;
};

struct PID_parameter
{
	float Kp;
	float Ki;
	float Kd;
};

struct Set_value        //pid�еĸ���Ŀ��ֵ    ������Ҳ���ڴ洢��������ۻ�
{
	float gyro_x;
	float gyro_y;
	float gyro_z;
	
	float pitch;
	float roll;
	float yaw;
	
	float height;
};



void flight_control(int COMorDMAmode, int COMmode);

void MPU6050_get_offset(int mode);

void Attitude_process(int COMorDMAmode, int COMmode);       //���㵱ǰ��̬

void Get_angle(struct Attitude_float *addr, float dt);

void duty_runout_adjustment(void);       //ռ�ձ��������

void flight_control_debug(int DebugMode, float set_value, int COMorDMAmode, int COMmode);     //�ɿ�PID����������Գ���   modeΪ���Զ���   set_valueΪĿ��ֵ

float limit(float target_value, float max, float min);    //ͨ���޷�����

#endif

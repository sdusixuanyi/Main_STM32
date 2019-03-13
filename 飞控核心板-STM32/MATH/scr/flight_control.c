#include "flight_control.h"

extern unsigned char raw_data[14];
extern short int translated_data[7];     //将6050的原始14个数据化为7个二进制数据，前三个是加速度分别在xyz上的分量，最后三个是角速度，中间的是温度

struct PID_parameter PID_pitch  = {1, 2, 1};  //姿态角PID
struct PID_parameter PID_roll   = {1, 3, 1};
struct PID_parameter PID_yaw    = {1, 1, 1};

struct PID_parameter PID_gyro_x = {1, 1, 1};  //角速度PID
struct PID_parameter PID_gyro_y = {1, 1, 1};
struct PID_parameter PID_gyro_z = {1, 1, 1};

struct PID_parameter PID_height = {1, 1, 1};  //高度PID

struct Set_value Set;

struct Attitude_int offset = {0,0,0, 0,0,0, 0,0,0};     //四旋翼静态的时候6050测得的数据
struct Attitude_float attitude;   //四旋翼当前的姿态数据

struct Quaternion NumQ = {1, 0, 0, 0};    //四元数
struct Vector_t GyroIntegError = {0};

struct Kalman_data kalman_accel_x = {0.02, 0.001, 0.543, 0};     //三轴的加速度卡尔曼滤波数据（从左到右为last_p, Q, R, output）
struct Kalman_data kalman_accel_y = {0.02, 0.001, 0.543, 0};
struct Kalman_data kalman_accel_z = {0.02, 0.001, 0.543, 0};

struct Set_value err[ERR_STORE_NUM] = { {0,0,0,  0,0,0,   0},
																					{0,0,0,  0,0,0,   0},
																					{0,0,0,  0,0,0,   0},
																					{0,0,0,  0,0,0,   0},
																					{0,0,0,  0,0,0,   0},
																					{0,0,0,  0,0,0,   0},
																					{0,0,0,  0,0,0,   0},
																					{0,0,0,  0,0,0,   0},
																					{0,0,0,  0,0,0,   0},
																					{0,0,0,  0,0,0,   0} };      //误差存储
																								
struct Set_value err_acc = {0,0,0,  0,0,0,  0};
																							
int store_pos = 0;      //该轮误差存储位置
																								
float motor_duty[4] = {STA_DUTY, STA_DUTY, STA_DUTY, STA_DUTY};   //X型 暂定编号：飞机平放头朝自己，俯视下逆时针给电机编号0123，01为头，23为尾
float motor_delta_duty[4] = {0,0,0,0};

void flight_control(int COMorDMAmode, int COMmode)
{
	int i;
	int read_position;               //计算隔N个数据计算时上一个读取数据的位置
  int period = 2;	
	
	float temp_delta_duty;           //中间变量，临时存放占空比增量
	double gra_ang_z;  //z轴和重力方向的夹角
	
	Attitude_process(COMorDMAmode, COMmode);     //解算姿态
	
	for(i = 0 ; i < 4 ; i++)   motor_delta_duty[i] = 0;    //清空占空比增量
	
	gra_ang_z = (atan( sqrt( (double)(attitude.accel_x*attitude.accel_x + attitude.accel_y*attitude.accel_y) ) / sqrt( (double)( attitude.accel_z*attitude.accel_z ) ) ) );
	attitude.height = attitude.height * (float)(cos(gra_ang_z));     //根据飞行器当前姿态将传感器测得距离转化为高度
	
	
	Set.pitch = 0;           //设定悬停时各项目标值
	Set.pitch = 0;	
	Set.gyro_z = 0;	
	Set.height = 1000;
	
	Set.pitch = limit(Set.pitch, LIM_PITCH_MAX, LIM_PITCH_MIN);
	Set.roll  = limit(Set.roll,  LIM_ROLL_MAX,  LIM_ROLL_MIN);
	Set.yaw   = limit(Set.yaw,   LIM_YAW_MAX,   LIM_YAW_MIN);
	Set.height= limit(Set.height, LIM_HEIGHT_MAX, LIM_HEIGHT_MIN);     //对目标高度进行限幅
	
	if(period > ERR_STORE_NUM)  period = ERR_STORE_NUM;   //防止数组越界
	
	if(store_pos - (period - 1) >= 0)   read_position = store_pos - (period - 1);   //计算读取位置（微分运算要用）
	else read_position = ERR_STORE_NUM - ((period - 1) - store_pos) + 1;
	
	
	//**************************************************高度PID***********************************************************
	err_acc.height = err_acc.height - err[store_pos].height;   //计算ERR_STORE_NUM个数据的误差和（减去即将被覆盖的误差）
	err[store_pos].height = Set.height - attitude.height;   //计算并存储高度偏差
	err_acc.height = err_acc.height + err[store_pos].height;   //计算ERR_STORE_NUM个数据的误差和（加上刚刚覆盖的新数据）
	
	temp_delta_duty = PID_height.Kp * err[store_pos].height
	                 +PID_height.Ki * err_acc.height
									 +PID_height.Kd * ( err[store_pos].height - err[read_position].height );
									 
	temp_delta_duty = limit(temp_delta_duty, LIM_DELTA_DUTY_HEIGHT_MAX, LIM_DELTA_DUTY_HEIGHT_MIN);     //高度占空比增量限幅
									 
	for(i = 0 ; i < 4 ; i ++)  motor_delta_duty[i] += temp_delta_duty;
	//********************************************************************************************************************
	
	
	//**************************************************角度PID***********************************************************
	err_acc.pitch -= err[store_pos].pitch;
	err[store_pos].pitch = Set.pitch - attitude.pitch;
	err_acc.pitch += err[store_pos].pitch;
	
	Set.gyro_y = PID_pitch.Kp * err[store_pos].pitch          //设定俯仰角角速度目标值
	            +PID_pitch.Ki * err_acc.pitch
							+PID_pitch.Kd * ( err[store_pos].pitch - err[read_position].pitch );
							
	Set.gyro_y = limit(Set.gyro_y, LIM_GYRO_Y_MAX, LIM_GYRO_Y_MIN);
	
	
	err_acc.roll -= err[store_pos].roll;
	err[store_pos].roll = Set.roll - attitude.roll;
	err_acc.roll += err[store_pos].roll;
	
	Set.gyro_x = PID_roll.Kp * err[store_pos].roll          //设定横滚角角速度目标值
	            +PID_roll.Ki * err_acc.roll
							+PID_roll.Kd * ( err[store_pos].roll - err[read_position].roll );
							
	Set.gyro_x = limit(Set.gyro_x, LIM_GYRO_X_MAX, LIM_GYRO_Y_MIN);
  
	Set.gyro_z = 0;
	Set.gyro_z = limit(Set.gyro_z, LIM_GYRO_Z_MAX, LIM_GYRO_Z_MIN);
	//*********************************************************************************************************************
	
	
	//**************************************************角速度PID**********************************************************
	err_acc.gyro_y -= err[store_pos].gyro_y;  //俯仰方向
	err[store_pos].gyro_y = Set.gyro_y - attitude.gyro_y;
	err_acc.gyro_y += err[store_pos].gyro_y;
	
	temp_delta_duty = PID_gyro_y.Kp * err[store_pos].gyro_y
	                 +PID_gyro_y.Ki * err_acc.gyro_y
									 +PID_gyro_y.Kd * ( err[store_pos].gyro_y - err[read_position].gyro_y);
									 
	temp_delta_duty = limit(temp_delta_duty, LIM_DELTA_DUTY_PITCH_MAX, LIM_DELTA_DUTY_PITCH_MIN);   //俯仰方向的PID增量限幅
										 
	motor_delta_duty[0] += temp_delta_duty;          //俯仰方向增量
	motor_delta_duty[1] += temp_delta_duty;
	motor_delta_duty[2] -= temp_delta_duty;
	motor_delta_duty[3] -= temp_delta_duty;


	
	err_acc.gyro_x -= err[store_pos].gyro_x;  //横滚方向
	err[store_pos].gyro_x = Set.gyro_x - attitude.gyro_x;
	err_acc.gyro_x += err[store_pos].gyro_x;
	
	temp_delta_duty = PID_gyro_x.Kp * err[store_pos].gyro_x
	                 +PID_gyro_x.Ki * err_acc.gyro_x
									 +PID_gyro_x.Kd * ( err[store_pos].gyro_x - err[read_position].gyro_x);
									 
	temp_delta_duty = limit(temp_delta_duty, LIM_DELTA_DUTY_ROLL_MAX, LIM_DELTA_DUTY_ROLL_MIN);   //横滚方向的PID增量限幅
									 
	motor_delta_duty[0] -= temp_delta_duty;          //横滚方向增量
	motor_delta_duty[1] += temp_delta_duty;
	motor_delta_duty[2] += temp_delta_duty;
	motor_delta_duty[3] -= temp_delta_duty;
	
	
	
	err_acc.gyro_z -= err[store_pos].gyro_z;  //偏航方向
	err[store_pos].gyro_z = Set.gyro_z - attitude.gyro_z;
	err_acc.gyro_z += err[store_pos].gyro_z;
	
	temp_delta_duty = PID_gyro_z.Kp * err[store_pos].gyro_z
	                 +PID_gyro_z.Ki * err_acc.gyro_z
									 +PID_gyro_z.Kd * ( err[store_pos].gyro_z - err[read_position].gyro_z);
									 
	temp_delta_duty = limit(temp_delta_duty, LIM_DELTA_DUTY_YAW_MAX, LIM_DELTA_DUTY_YAW_MIN);   //偏航方向的PID增量限幅
									 
	motor_delta_duty[0] += temp_delta_duty;          //偏航方向轴增量
	motor_delta_duty[1] -= temp_delta_duty;
	motor_delta_duty[2] += temp_delta_duty;
	motor_delta_duty[3] -= temp_delta_duty;
	//**********************************************************************************************************************
	
	for(i = 0 ; i < 4 ; i++)     motor_duty[i] += motor_delta_duty[i];      //计算最终输出
	
	duty_runout_adjustment();
	
	for(i = 0 ; i < 4 ; i++)     motor_duty[i] = limit(motor_duty[i], DUTY_MAX, DUTY_MIN);
	
	store_pos ++;
	if(store_pos == ERR_STORE_NUM)   store_pos = 0;
}

void MPU6050_get_offset(int mode)
{
	int i;
	for(i = 0 ; i < 100 ; i ++)
	{
		MPU6050_get_data(raw_data, mode);
		
		MPU6050_data_translation(raw_data, translated_data);
		
		offset.gyro_x += translated_data[4];
		offset.gyro_y += translated_data[5];
		offset.gyro_z += translated_data[6];
	}
	
	offset.gyro_x = offset.gyro_x / 100;     //得到角速度的静态测量误差
	offset.gyro_y = offset.gyro_y / 100;
	offset.gyro_z = offset.gyro_z / 100;
	
	for(i = 0 ; i < 100 ; i ++)
	{
		MPU6050_get_data(raw_data, mode);
		
		MPU6050_data_translation(raw_data, translated_data);
		
		translated_data[4] -= offset.gyro_x;         //角速度减去静态偏差
		translated_data[5] -= offset.gyro_y;
		translated_data[6] -= offset.gyro_z;
		
		kalman_filter(&kalman_accel_x, (float)translated_data[0]);    //对三轴加速度进行卡尔曼滤波
		kalman_filter(&kalman_accel_y, (float)translated_data[1]);
		kalman_filter(&kalman_accel_z, (float)translated_data[2]);
		
		attitude.accel_x = kalman_accel_x.output;      //将滤波的结果存入当前姿态结构体
		attitude.accel_y = kalman_accel_y.output;
		attitude.accel_z = kalman_accel_z.output;
		
		attitude.gyro_x = translated_data[4];
		attitude.gyro_y = translated_data[5];
		attitude.gyro_z = translated_data[6];	
		
		Get_angle(&attitude, 0.00626);
		
		offset.pitch += attitude.pitch;
		offset.roll  += attitude.roll;
	}
	
	offset.pitch = offset.pitch / 100;
	offset.roll  = offset.roll / 100;
}

void Attitude_process(int COMorDMAmode, int COMmode)
{
	if (COMorDMAmode == COMMON_READ) MPU6050_get_data(raw_data, COMmode);      //常规读取
	
	MPU6050_data_translation(raw_data, translated_data);
	
	if (COMorDMAmode == DMA_READ)    MPU6050_dma_read(MPU6050_ADDRESS, 0x3B);   //DMA方式读取
	
	translated_data[4] -= offset.gyro_x;         //角速度减去静态偏差
	translated_data[5] -= offset.gyro_y;
	translated_data[6] -= offset.gyro_z;
	
	kalman_filter(&kalman_accel_x, (float)translated_data[0]);    //对三轴加速度进行卡尔曼滤波
	kalman_filter(&kalman_accel_y, (float)translated_data[1]);
	kalman_filter(&kalman_accel_z, (float)translated_data[2]);
	
	attitude.accel_x = kalman_accel_x.output;      //将滤波的结果存入当前姿态结构体
	attitude.accel_y = kalman_accel_y.output;
	attitude.accel_z = kalman_accel_z.output;
	
	attitude.gyro_x = translated_data[4];      //此时还是二进制数
	attitude.gyro_y = translated_data[5];
	attitude.gyro_z = translated_data[6];	
	
	Get_angle(&attitude, 0.00626);
	
	attitude.pitch -= offset.pitch;    //得到的姿态角减去静态偏差
	attitude.roll  -= offset.roll;
	
	attitude.gyro_x = attitude.gyro_x * Gyro_G;         //转化为十进制
	attitude.gyro_y = attitude.gyro_y * Gyro_G;
	attitude.gyro_z = attitude.gyro_z * Gyro_G;
}

void Get_angle(struct Attitude_float *addr, float dt)
{
	struct Vector_t Gravity,Acc,Gyro,AccGravity;
	static float KpDef = 0.8f ; //??????
	static float KiDef = 0.0003f; 
  
	float q0_t,q1_t,q2_t,q3_t;
	float NormQuat; 
	float HalfTime = dt * 0.5f;
	
	float vecxZ;
	float vecyZ;
	float veczZ;
	float yaw_G;

	Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);                
	Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);              
	Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);  
	//
 	NormQuat = Q_rsqrt( squa(addr->accel_x)+ squa(addr->accel_y) +squa(addr->accel_z) );
// 	//
	Acc.x = addr->accel_x * NormQuat; //
	Acc.y = addr->accel_y * NormQuat;  
	Acc.z = addr->accel_z * NormQuat;  

// 	//
	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

	GyroIntegError.x += AccGravity.x * KiDef;
	GyroIntegError.y += AccGravity.y * KiDef;
	GyroIntegError.z += AccGravity.z * KiDef;
// 	//
	Gyro.x = (addr->gyro_x) * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//???,,????????????
	Gyro.y = (addr->gyro_y) * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
	Gyro.z = (addr->gyro_z) * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;    
// 	//
// 	//
	q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
	q1_t = ( NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
	q2_t = ( NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
	q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

	NumQ.q0 += q0_t; //
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
// 	//
	NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3)); //
	NumQ.q0 *= NormQuat; //
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;  
// 	
//
	vecxZ = 2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3 ;//
	vecyZ = 2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;//
	veczZ = NumQ.q0 * NumQ.q0 - NumQ.q1 * NumQ.q1 - NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3;  //
// 	//float veczZ =  1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2;  //
// 	#ifdef  YAW_GYRO
// 			*(float *)pAngE = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
// 	#else   //
			yaw_G = addr->gyro_z * Gyro_G;//
	
	if((yaw_G > 1.0f) || (yaw_G < -1.0f)) //
	{
			addr->yaw  += yaw_G * dt;//
	}
// 	#endif
//  
	addr->pitch  =  asin(vecxZ)* RtA;   //
	addr->roll  = atan2f(vecyZ,veczZ) * RtA;  //
	//NormAccz = pMpu->accX* vecxZ + pMpu->accY * vecyZ + pMpu->accZ * veczZ;  //
	//HeightInfo.Z_Acc = (NormAccz - 8192) * 0.1196f;
}

void duty_runout_adjustment()
{
	int run_out_num[2] = {0,0};      //0：超出上限，1：超出下限
	int i;
  float temp_max = DUTY_MIN, temp_min = DUTY_MAX;       //求最大最小值时的中间变量
  int max , min;                                        //最大最小值的数组编号
  float ratio = 1;
  
	
	for(i = 0 ; i < 4 ; i ++)
	{
		if(motor_duty[i] > DUTY_MAX)   run_out_num[0] ++;       //统计占空比溢出个数
		if(motor_duty[i] < DUTY_MIN)   run_out_num[1] ++;
		
		if(motor_duty[i] > temp_max)    { max = i;  temp_max = motor_duty[i];}             //求四个占空比输出中的最大最小值
		if(motor_duty[i] < temp_min)    { min = i;  temp_min = motor_duty[i];}
	}
	
	if(run_out_num[0] >= 1)      //超出最大占空比
	{
		ratio = (DUTY_MAX - DUTY_MIN) / (motor_duty[max] - DUTY_MIN);         //对超出最小占空比的部分进行缩放
		
		for(i = 0 ; i < 4 ; i ++)
		{
			motor_duty[i] = DUTY_MIN + ratio * (motor_duty[i] - DUTY_MIN);
		}
		motor_duty[max] = DUTY_MAX;
	}
	
	if(run_out_num[1] >= 1)    //低于最小占空比
	{
		ratio = (DUTY_MAX - DUTY_MIN) / (DUTY_MAX - motor_duty[min]);         //对低于最大占空比的部分进行缩放
		
		for(i = 0 ; i < 4 ; i ++)
		{
			motor_duty[i] = DUTY_MAX - ratio * (DUTY_MAX - motor_duty[i]);
		}
		motor_duty[min] = DUTY_MIN;
	}
}


void flight_control_debug(int DebugMode, float set_value, int COMorDMAmode, int COMmode)
{
	//int height_flag = 0;
	int gyro_x_flag = 0,
	    gyro_y_flag = 0,
	    gyro_z_flag = 0,
	    roll_flag   = 0,
	    pitch_flag  = 0,
	    yaw_flag    = 0;
	
	int i;
	int read_position;               //计算隔N个数据计算时上一个读取数据的位置
  int period = 2;	
	
	float temp_delta_duty;           //中间变量，临时存放占空比增量
	double gra_ang_z;  //z轴和重力方向的夹角
	
	Attitude_process(COMorDMAmode, COMmode);
	
	for(i = 0 ; i < 4 ; i++)   motor_delta_duty[i] = 0;    //清空占空比增量
	
	gra_ang_z = (atan( sqrt( (double)(attitude.accel_x*attitude.accel_x + attitude.accel_y*attitude.accel_y) ) / sqrt( (double)( attitude.accel_z*attitude.accel_z ) ) ) );
	attitude.height = attitude.height * (float)(cos(gra_ang_z));     //根据飞行器当前姿态将传感器测得距离转化为高度
	
	if(period > ERR_STORE_NUM)  period = ERR_STORE_NUM;   //防止数组越界
	
	if(store_pos - (period - 1) >= 0)   read_position = store_pos - (period - 1);   //计算读取位置（微分运算要用）
	else read_position = ERR_STORE_NUM - ((period - 1) - store_pos) + 1;
	
	switch (DebugMode)
	{
		case DEBUG_MODE_GYRO_X : gyro_x_flag = 1; Set.gyro_x = set_value; break;
		case DEBUG_MODE_GYRO_Y : gyro_y_flag = 1; Set.gyro_y = set_value; break;
		case DEBUG_MODE_GYRO_Z : gyro_z_flag = 1; Set.gyro_z = set_value; break;
		
		case DEBUG_MODE_ROLL   : roll_flag   = 1; gyro_x_flag = 1; Set.roll  = set_value; break;
		case DEBUG_MODE_PITCH  : pitch_flag  = 1; gyro_y_flag = 1; Set.pitch = set_value; break;
		case DEBUG_MODE_YAW    : yaw_flag    = 1; gyro_z_flag = 1; Set.yaw   = set_value; break;
	}
	
	//**********************************************X轴调试（roll）*************************************************************
	if(roll_flag)
	{
		err_acc.roll -= err[store_pos].roll;
		err[store_pos].roll = Set.roll - attitude.roll;
		err_acc.roll += err[store_pos].roll;
		
		Set.gyro_x = PID_roll.Kp * err[store_pos].roll          //设定横滚角角速度目标值
								+PID_roll.Ki * err_acc.roll
								+PID_roll.Kd * ( err[store_pos].roll - err[read_position].roll );
		
		Set.gyro_x = limit(Set.gyro_x, LIM_GYRO_X_MAX, LIM_GYRO_Y_MIN);
	}
	
	if(gyro_x_flag)
	{
		err_acc.gyro_x -= err[store_pos].gyro_x;  //横滚方向
		err[store_pos].gyro_x = Set.gyro_x - attitude.gyro_x;
		err_acc.gyro_x += err[store_pos].gyro_x;
		
		temp_delta_duty = PID_gyro_x.Kp * err[store_pos].gyro_x
										 +PID_gyro_x.Ki * err_acc.gyro_x
										 +PID_gyro_x.Kd * ( err[store_pos].gyro_x - err[read_position].gyro_x);
		
		temp_delta_duty = limit(temp_delta_duty, LIM_DELTA_DUTY_ROLL_MAX, LIM_DELTA_DUTY_ROLL_MIN);   //横滚方向的PID增量限幅
										 
		motor_delta_duty[0] -= temp_delta_duty;          //横滚方向增量
		motor_delta_duty[1] += temp_delta_duty;
		motor_delta_duty[2] += temp_delta_duty;
		motor_delta_duty[3] -= temp_delta_duty;
	}
	//**************************************************************************************************************************
	
	//***********************************************Y轴调试（pitch）***********************************************************
	if(pitch_flag)
	{
		err_acc.pitch -= err[store_pos].pitch;
		err[store_pos].pitch = Set.pitch - attitude.pitch;
		err_acc.pitch += err[store_pos].pitch;
		
		Set.gyro_y = PID_pitch.Kp * err[store_pos].pitch          //设定俯仰角角速度目标值
								+PID_pitch.Ki * err_acc.pitch
								+PID_pitch.Kd * ( err[store_pos].pitch - err[read_position].pitch );
		
		Set.gyro_y = limit(Set.gyro_y, LIM_GYRO_Y_MAX, LIM_GYRO_Y_MIN);
	}
	
	if(gyro_y_flag)
	{
		err_acc.gyro_y -= err[store_pos].gyro_y;  //俯仰方向
		err[store_pos].gyro_y = Set.gyro_y - attitude.gyro_y;
		err_acc.gyro_y += err[store_pos].gyro_y;
		
		temp_delta_duty = PID_gyro_y.Kp * err[store_pos].gyro_y
										 +PID_gyro_y.Ki * err_acc.gyro_y
										 +PID_gyro_y.Kd * ( err[store_pos].gyro_y - err[read_position].gyro_y);
		
		temp_delta_duty = limit(temp_delta_duty, LIM_DELTA_DUTY_PITCH_MAX, LIM_DELTA_DUTY_PITCH_MIN);   //俯仰方向的PID增量限幅
										 
		motor_delta_duty[0] += temp_delta_duty;          //俯仰方向增量
		motor_delta_duty[1] += temp_delta_duty;
		motor_delta_duty[2] -= temp_delta_duty;
		motor_delta_duty[3] -= temp_delta_duty;
	}
	//**************************************************************************************************************************
	
	//***********************************************Z轴调试（yaw）*************************************************************
	if(yaw_flag)
	{
		Set.gyro_z = 0;
		
		Set.gyro_z = limit(Set.gyro_z, LIM_GYRO_Z_MAX, LIM_GYRO_Z_MIN);
	}
	
	if(gyro_z_flag)
	{
		err_acc.gyro_z -= err[store_pos].gyro_z;  //偏航方向
		err[store_pos].gyro_z = Set.gyro_z - attitude.gyro_z;
		err_acc.gyro_z += err[store_pos].gyro_z;
		
		temp_delta_duty = PID_gyro_z.Kp * err[store_pos].gyro_z
										 +PID_gyro_z.Ki * err_acc.gyro_z
										 +PID_gyro_z.Kd * ( err[store_pos].gyro_z - err[read_position].gyro_z);
		
		temp_delta_duty = limit(temp_delta_duty, LIM_DELTA_DUTY_YAW_MAX, LIM_DELTA_DUTY_YAW_MIN);   //偏航方向的PID增量限幅
										 
		motor_delta_duty[0] += temp_delta_duty;          //偏航方向轴增量
		motor_delta_duty[1] -= temp_delta_duty;
		motor_delta_duty[2] += temp_delta_duty;
		motor_delta_duty[3] -= temp_delta_duty;
	}
	//**************************************************************************************************************************
	
	for(i = 0 ; i < 4 ; i++)     motor_duty[i] += motor_delta_duty[i];      //计算最终输出
	
	duty_runout_adjustment();
	
	for(i = 0 ; i < 4 ; i++)     motor_duty[i] = limit(motor_duty[i], DUTY_MAX, DUTY_MIN);
	
	store_pos ++;
	if(store_pos == ERR_STORE_NUM)   store_pos = 0;
}


float limit(float target_value, float max, float min)
{
	if(min > max) return target_value;
	
	if(max != 0 && target_value > max)  target_value = max;
	if(min != 0 && target_value < min)  target_value = min;
	
	return target_value;
}

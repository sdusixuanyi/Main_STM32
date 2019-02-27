#include "kalman.h"

void kalman_filter(struct Kalman_data * addr, float input)
{
	float K;     //卡尔曼增益
	float now_p;  //当前时刻预测的系统协方差
	
	now_p = (*addr).last_p + (*addr).Q;      //预测协方差方程
	
	K = ( now_p ) / ( now_p + (*addr).R);    //计算卡尔曼增益
	
	(*addr).output = (*addr).output + K * (input - (*addr).output);    //更新最优值
	
	(*addr).last_p = (1 - K) * now_p;
}

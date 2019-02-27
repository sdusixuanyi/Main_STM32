#ifndef _KALMAN_H_
#define _KALMAN_H_

#include "stm32f10x.h"
#include "math.h"

struct Kalman_data      //卡尔曼滤波所用的结构体
{
	float last_p;    //上一时刻的系统协方差矩阵
	float Q;         //过程噪声协方差矩阵，取值影响不大
	float R;         //测量对象噪声协方差矩阵，需要用实验法确定最佳值
	float output;    //卡尔曼滤波后的输出
	//float now_p;     //这一时刻预测的系统协方差矩阵
};

void kalman_filter(struct Kalman_data *addr, float input);     //移植自中科浩电的一维卡尔曼

#endif

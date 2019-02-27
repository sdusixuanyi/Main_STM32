#ifndef _KALMAN_H_
#define _KALMAN_H_

#include "stm32f10x.h"
#include "math.h"

struct Kalman_data      //�������˲����õĽṹ��
{
	float last_p;    //��һʱ�̵�ϵͳЭ�������
	float Q;         //��������Э�������ȡֵӰ�첻��
	float R;         //������������Э���������Ҫ��ʵ�鷨ȷ�����ֵ
	float output;    //�������˲�������
	//float now_p;     //��һʱ��Ԥ���ϵͳЭ�������
};

void kalman_filter(struct Kalman_data *addr, float input);     //��ֲ���пƺƵ��һά������

#endif

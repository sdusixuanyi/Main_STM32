#include "kalman.h"

void kalman_filter(struct Kalman_data * addr, float input)
{
	float K;     //����������
	float now_p;  //��ǰʱ��Ԥ���ϵͳЭ����
	
	now_p = (*addr).last_p + (*addr).Q;      //Ԥ��Э�����
	
	K = ( now_p ) / ( now_p + (*addr).R);    //���㿨��������
	
	(*addr).output = (*addr).output + K * (input - (*addr).output);    //��������ֵ
	
	(*addr).last_p = (1 - K) * now_p;
}

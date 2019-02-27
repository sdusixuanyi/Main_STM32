#ifndef _MYMATH_H_
#define _MYMATH_H_

#include "math.h"

#define squa( Sq )    (((float)Sq)*((float)Sq))

#define M_PI  3.1415926535
#define RtA  57.2957795f
#define AtR  0.0174532925f
#define Gyro_G  0.03051756f * 2     //ÿ�仯һ��LSBλ��Ӧ�ĽǶ�
#define Gyro_Gr  0.0005326f * 2     //ÿ�仯һ��LSB��Ӧ�Ļ���
#define PI_2  1.570796f

float Q_rsqrt(float number);

#endif

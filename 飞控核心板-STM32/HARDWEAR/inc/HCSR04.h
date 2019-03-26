#ifndef _HCSR04_H
#define _HCSR04_H
//#include "sys.h"

#include "headfile.h"


extern uint8_t HCSR04_Error,HCSR04_OK,HCSR04_RunFlag,HCSR04_Update;
extern float HCSR04_Distance;
void HCSR04_Init(void);
void HCSR04_Run(void);
float HCSR04_Get_Distance(int time);
void HCSR04_GPIO_Configuration(void);
void HCSR04_EXTI_Configuration(void);
void TIM2_Int_Init(u16 arr,u16 psc);

#endif


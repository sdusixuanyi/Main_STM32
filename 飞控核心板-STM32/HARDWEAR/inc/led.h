#ifndef __LED_H
#define __LED_H	 
//#include "sys.h"

#include "headfile.h"

void LED_Init(void);//≥ı ºªØ
void LED_ON(u8 num);//num=0\1\2\3,turn on the right LED;4 turn on all led;else delay_ms(10)
void LED_OFF(u8 num);//num=0\1\2\3,turn off the right LED;4 turn off all led;else delay_ms(10)

		 				    
#endif

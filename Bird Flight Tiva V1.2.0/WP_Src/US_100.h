#ifndef _US_100_H_
#define _US_100_H_


//US100
#define User_Height_Max  150//用户使用的最大高度值，单位为cm，需要流出一定裕度给气压计/超声波切换过渡



#define US_100_Distance_CMD    0x55
#define US_100_Temperature_CMD 0x50

void US_100_Statemachine(void);



extern uint8_t US_100_Update_Flag;
extern float US_Distance;
#endif



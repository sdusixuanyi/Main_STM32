/*----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于无名科创团队，无名科创团队将飞控程序源码提供给购买者，
*               购买者要为无名科创团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载， 
*               更不能以此销售牟利，如发现上述行为，无名科创团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名科创开源飞控QQ群：540707961
*               CSDN博客：http://blog.csdn.net/u011992534
*               优酷ID：NamelessCotrun无名小哥
*               B站教学视频：https://space.bilibili.com/67803559/#/video
*               客户使用心得、改进意见征集贴：http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               百度贴吧:无名科创开源飞控
*               修改日期:2018/8/30
*               版本：慧飞者——V1.0.1
*               版权所有，盗版必究。
*               Copyright(C) 武汉科技大学无名科创团队 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#ifndef _CONTROL_LAND_H
#define _CONTROL_LAND_H

#define Faraway_Distance 600//距离home很远，计为A区域
#define Near_Distance 150//距离home较远，计为B区域
#define Arrive_Distance 0//达到home点，计为C区域


#define First_Nav_Rate   100//从A->B，开启一级巡航速度，单位cm
#define Second_Nav_Rate  80//从B->C，开启二级巡航速度，单位cm
#define Third_Nav_Rate   60//处于C点附近，开启三级巡航速度, 单位cm
#define Default_Nav_rate 50//默认巡航速度


#define Nav_Safety_Height 800//距离返航点较远时，返航所需的安全巡航高度，单位cm
#define Nav_Climb_Rate  80//当距离返航点较远时，且当前高度小于Nav_Safety_Height时，原地上升时的速度
#define Nav_Decline_Rate -40//当返航至home点正上方时，原地下降至地面时的速度
#define Nav_Rapid_Decline_Rate -80//快速下降至地面时的速度


#define Nav_Transition_Period 400//5ms*400=2S


void land_state_check(void);
void land_reset(void);
void landon_earth_check(void);
void land_run(void);
bool GPS_ok(void);
bool land_althold(float taret_climb_rate,float target_climb_alt);


#endif


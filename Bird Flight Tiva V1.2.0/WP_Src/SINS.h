#ifndef __SINS_H_
#define __SINS_H_

typedef struct{
  // by making alt 24 bit we can make p1 in a command 16 bit,
  // allowing an accurate angle in centi-degrees. This keeps the
  // storage cost per mission item at 15 bytes, and allows mission
  // altitudes of up to +/- 83km
  int32_t alt:24; ///< param 2 - Altitude in centimeters (meters * 100)
  int32_t lat;    ///< param 3 - Lattitude * 10**7
  int32_t lng;    ///< param 4 - Longitude * 10**7
}Location;



float longitude_scale(Location loc);
float get_distance(Location loc1,Location loc2);
void  GPSData_Sort(void);
void  Set_GPS_Home(void);
void  SINS_Prepare(void);
void Strapdown_INS_High_Kalman(void);
void Strapdown_INS_Reset(SINS *Ins,uint8_t Axis,float Pos_Target,float Vel_Target);
void KalmanFilter_Horizontal(void);
extern SINS NamelessQuad;
extern SINS Origion_NamelessQuad;
extern uint8_t Filter_Defeated_Flag;
extern Vector2f SINS_Accel_Body;
extern uint8 GPS_Home_Set;
extern Location GPS_Home;//初始定位成功点信息
extern Location GPS_Present;//当前位置点信息
extern float Acceleration_Feedback[3];
extern float Observation_Altitude;


extern uint8_t GPS_Home_Set;
extern float Declination;//地球磁偏角
#endif


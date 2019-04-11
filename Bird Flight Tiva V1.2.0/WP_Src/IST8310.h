#ifndef __IST8310_H
#define __IST8310_H

void IST8310_Init(void);
uint8_t IST8310_Read_Data(vector3f *mag);
extern Vector2f magn;
extern Vector3f mag_offset;
extern Butter_Parameter Mag_Parameter;
#endif


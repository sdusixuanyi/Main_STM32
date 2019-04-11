#include "Headfile.h"
#include "myiic.h"
#include "mpu6050.h"
Butter_Parameter Ins_Accel_Parameter;
Butter_Parameter Accel_Parameter;
Butter_Parameter Gyro_Parameter;
Butter_Parameter Calibrate_Parameter;
void Init_MPU6050(void)//MPU6050��ʼ��
{
  i2cWrite(MPU_ADRESS,PWR_MGMT_1  , 0x00);//�ر������ж�,�������
  i2cWrite(MPU_ADRESS,SMPLRT_DIV  , 0x00); // sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz
  i2cWrite(MPU_ADRESS,MPU_CONFIG  , 0x04); //�ڲ���ͨ�˲�Ƶ�ʣ����ٶȼ�21hz��������20hz
  i2cWrite(MPU_ADRESS,GYRO_CONFIG , 0x08);//500deg/s
  i2cWrite(MPU_ADRESS,ACCEL_CONFIG, 0x10);// Accel scale 8g (4096 LSB/g)
  delay_ms(500);
  IMU_Calibration();
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 30,&Gyro_Parameter);//��̬���ٶȷ����˲�����
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 60,&Accel_Parameter);//��̬����Ӽ������˲�ֵ50
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 2,&Calibrate_Parameter);//������У׼�Ӽ��˲�ֵ
  Set_Cutoff_Frequency(Imu_Sampling_Freq, 30,&Ins_Accel_Parameter);//INS�Ӽ��˲�ֵ
}


void MPU6050_Read_Data(vector3f *gyro,vector3f *accel)//��ȡMPU6050����
{
  accel->x=Double_ReadI2C(MPU_ADRESS,ACCEL_XOUT_H);
  accel->y=Double_ReadI2C(MPU_ADRESS,ACCEL_YOUT_H);
  accel->z=Double_ReadI2C(MPU_ADRESS,ACCEL_ZOUT_H);
  gyro->x=Double_ReadI2C(MPU_ADRESS,GYRO_XOUT_H);
  gyro->y=Double_ReadI2C(MPU_ADRESS,GYRO_YOUT_H);
  gyro->z=Double_ReadI2C(MPU_ADRESS,GYRO_ZOUT_H);
}



Vector3f gyro_offset;
s32 g_Gyro_xoffset = 0, g_Gyro_yoffset = 0, g_Gyro_zoffset = 0;
void IMU_Calibration(void)//��������ƫ�궨
{
  u8 i;
  delay_ms(500);
  for (i = 0; i < 100; i++)			//��������30�Σ�һ����ʱ30*3=90ms
  {
    g_Gyro_xoffset +=Double_ReadI2C(MPU_ADRESS,GYRO_XOUT_H);;
    g_Gyro_yoffset +=Double_ReadI2C(MPU_ADRESS,GYRO_YOUT_H);;
    g_Gyro_zoffset +=Double_ReadI2C(MPU_ADRESS,GYRO_ZOUT_H);;
    delay_ms(10);
  }
  gyro_offset.x =(g_Gyro_xoffset/100);//�õ��궨ƫ��
  gyro_offset.y =(g_Gyro_yoffset/100);
  gyro_offset.z =(g_Gyro_zoffset/100);
}


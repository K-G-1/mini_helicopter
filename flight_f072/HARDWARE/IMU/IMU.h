#ifndef __IMU_H
#define	__IMU_H
#include "stm32f0xx.h"



struct _angle{
        float pitch;
		float roll;
        float yaw;};
typedef struct 
{
    int16_t x;
    int16_t y;
    int16_t z;  
}T_int16_xyz;

struct IMU_DATA
{
    T_int16_xyz mpu6500_dataacc1;
    T_int16_xyz mpu6500_dataacc2;
    T_int16_xyz mpu6500_datagyr1;
    T_int16_xyz mpu6500_datagyr2;
};

extern struct _angle angle;
extern struct IMU_DATA imu_data;
void Prepare_Data(void);
void Prepare_6050_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void IMUupdate1(float gx, float gy, float gz, float ax, float ay, float az);

#endif














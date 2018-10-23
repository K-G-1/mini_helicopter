#ifndef __IMU_H
#define	__IMU_H
#include "stm32f0xx_hal.h"


#define RtA 		57.324841f		//  180/3.1415  弧度转化为角度		
#define AtR    	0.0174533f		//  1/RtA             RtA倒数		
#define Acc_G 	0.0011963f		//  1/32768/4/9.8     加速度量程为4G		int型1/(2^15)/4g
#define Gyro_G 	0.03051756f	//  1/32768/1000      陀螺仪初始化的量程为 +—1000			1/(2^15)/4g单位为度/秒
#define Gyro_Gr	0.0005327f  //  1/32768/1000/57.3 将上面的单位转化为弧度每秒
#define Gyro_G_x  0.06103f       //0.061036 //   4000/65536  +-2000


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














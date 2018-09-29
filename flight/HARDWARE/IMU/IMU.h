#ifndef __IMU_H
#define	__IMU_H
#include "stm32f10x.h"


#define RtA 		57.324841		//  180/3.1415  弧度转化为角度		
#define AtR    	0.0174533		//  1/RtA             RtA倒数		


struct _angle{
        float pitch;
		float roll;
        float yaw;};


extern struct _angle angle;

void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void IMUupdate1(float gx, float gy, float gz, float ax, float ay, float az);

#endif














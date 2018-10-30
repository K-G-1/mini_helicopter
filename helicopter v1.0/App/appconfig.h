#ifndef __appconfig_H
#define __appconfig_H

#include "stm32f10x.h"

typedef unsigned char uintbl;

typedef struct
{
	float rol;
	float pit;
	float yaw;
}T_float_angle;

typedef struct
{
	float X;
	float Y;
	float Z;
}T_float_xyz;

typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
	int16_t LastX;
	int16_t LastY;
	int16_t LastZ;
}T_int16_xyz;

typedef struct
{
	int32_t X;
	int32_t Y;
	int32_t Z;
}T_int32_xyz;



typedef struct int16_rcget
{
	int16_t ROLL;
	int16_t PITCH;
	int16_t THROTTLE;
	int16_t YAW;
	int16_t AUX1;
	int16_t AUX2;
	int16_t AUX3;
	int16_t AUX4;
	int16_t AUX5;
	int16_t AUX6;
}T_RC_Data;

typedef struct
{
	unsigned char ARMED;
}T_RC_Control;

typedef struct PID
{
	float P;
	float PwmOutP;
	float I;
	float PwmOutI;
	float D;
	float PwmOutD;
	float IMAX;
	float OUT;
	float ExternalP;
	float ExternalI;
	float InternalP;
	float InternalI;
	float InternalD;
	float ExternalOut;
	float InternalOut;
}T_PID;

#define BYTE0(dwTemp)       (*(unsigned char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((unsigned char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((unsigned char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((unsigned char *)(&dwTemp) + 3))



#endif

#ifndef __PARAM_H
#define __PARAM_H

#include "stm32f10x.h"
#include "Drv_flash.h"
//#include "ANO_PID.h"
#include "data.h"
/////////////////////////////////////////////////
#define FIRST_INIT_FLAG 		0XAA

typedef struct
{
	s32 kp;			 //????
	s32 ki;			 //????
	s32 kd;		 	 //????

}PID_param_st_pk; 

struct _save_param_st_pk{	
	u8 firstintiflag;
	u16 hardware;
	u16 software;
	u8 sensor_type;
	
	_xyz_f_st_pk acc_offset;//3?float 12??
	_xyz_f_st_pk gyr_offset;
	_xyz_f_st_pk vec_3d_offset;
	
	float gyr_temprea_offset;
	float acc_temprea_offset;
	
	PID_param_st_pk PID_ct4;
	PID_param_st_pk PID_ct3;
	
	PID_param_st_pk PID_rol; //12??,3?float
	PID_param_st_pk PID_pit;
	PID_param_st_pk PID_yaw;
	
	PID_param_st_pk PID_rol_s; //12??,3?float
	PID_param_st_pk PID_pit_s;
	PID_param_st_pk PID_yaw_s;
	
	PID_param_st_pk PID_hs;
	
};

/////////////////////////////////////////////////
extern struct _save_param_st_pk ANO_Param;

void ANO_Param_Init(void);
void ANO_Param_Read(void);
void ANO_Param_Save(void);
void PID_Save_Overtime(u16 ms,u16 dTms);//PID??????
#endif


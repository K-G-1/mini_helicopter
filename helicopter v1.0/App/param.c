/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * ??		 :????
 * ???  :ANO_Param.cpp
 * ??    :???????
 * ??    :www.anotc.com
 * ??    :anotc.taobao.com
 * ??Q? :190169595
**********************************************************************************/
#include "Param.h"
#include "MPU6050.h"
#include "control.h"

struct _save_param_st_pk ANO_Param;

void ANO_Param_Init()
{
	ANO_Param.firstintiflag = FIRST_INIT_FLAG;
	ANO_Param.hardware = 200;
	ANO_Param.software = 100;
	
	ANO_Param.PID_rol.kp = 8000;
	ANO_Param.PID_rol.ki = 10;
	ANO_Param.PID_rol.kd = 0;

	ANO_Param.PID_pit.kp = 8000;
	ANO_Param.PID_pit.ki = 10;
	ANO_Param.PID_pit.kd = 0;
	
	ANO_Param.PID_yaw.kp = 00;
	ANO_Param.PID_yaw.ki = 00;
	ANO_Param.PID_yaw.kd = 00;
	
	ANO_Param.PID_rol_s.kp = 260;
	ANO_Param.PID_rol_s.ki = 10;
	ANO_Param.PID_rol_s.kd = 20;
	
	ANO_Param.PID_pit_s.kp = 260;
	ANO_Param.PID_pit_s.ki = 10;
	ANO_Param.PID_pit_s.kd = 20;
	
	ANO_Param.PID_yaw_s.kp = 6000;
	ANO_Param.PID_yaw_s.ki = 00;
	ANO_Param.PID_yaw_s.kd = 1000;
	
	ANO_Param.PID_hs.kp = 00;
	ANO_Param.PID_hs.ki = 00;
	ANO_Param.PID_hs.kd = 00;

	ANO_Param_Save();
}

void ANO_Param_Read(void)
{
	ANO_Flash_Read((u8 *)(&ANO_Param),sizeof(ANO_Param));
	if(ANO_Param.firstintiflag != FIRST_INIT_FLAG)//???????
	{
		ANO_Param_Init();
	}
//	if(ANO_Param.firstintiflag != FIRST_INIT_FLAG)//???????
//	{
//		while(1);
//	}
}

void ANO_Param_Save(void)
{
	ANO_Flash_Write((u8 *)(&ANO_Param),sizeof(ANO_Param));
	g_v_PIDInit();
}

u16 save_pid_en = 0;
void PID_Save_Overtime(u16 ms,u16 dTms)
{
	if(save_pid_en!=0)
	{
		save_pid_en++;
	}
	
	if(save_pid_en>=(ms/dTms))
	{
		ANO_Param_Save();
		save_pid_en = 0;
	}

}







#ifndef __control_H
#define __control_H


#include "appconfig.h"

extern T_int16_xyz	 g_t_Gyro_CMS;//�����ۺϺ�Ĵ���������
extern T_int16_xyz	 g_t_Accel_CMS;
extern T_int16_xyz	 g_t_AccelAVG_CMS;
extern T_float_angle g_t_AttAngle_CMS;	//ATT�������������̬��
extern vs32			 g_vs32_Alt_CMS;
extern T_RC_Data 	 g_t_Rc_D_CMS;		//ң��ͨ������
extern T_RC_Control	 g_t_Rc_C_CMS;		//ң�ع�������
extern T_PID  		 g_t_PidRol_CMS;			//Rol pid data
extern T_PID  		 g_t_PidPit_CMS;			//Pit pid data
extern T_PID  		 g_t_PidYaw_CMS;			//Yaw pid data		
extern int16_t temp_rolpwmP,temp_rolpwmI,temp_pitpwmP,temp_pitpwmI;
void g_v_PIDInit(void);					
void g_v_GetMpuOffset(void);				
void g_v_GetMpuValue(void);
void g_v_UpdataPositioning(void);
void g_v_Control(void);
void g_v_ControlDublePID(void);
void g_v_ControlDublePID2(void);
#endif

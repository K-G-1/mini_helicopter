#ifndef _IMU_H_
#define _IMU_H_

#include "appconfig.h"

extern float 	AngleOffset_Rol,AngleOffset_Pit; 

void g_v_LoopFilter(T_int16_xyz *acc_in,T_int16_xyz *acc_out);
void g_v_ImuUpdate(T_int16_xyz *gyr, T_int16_xyz *acc, T_float_angle *angle);


#endif

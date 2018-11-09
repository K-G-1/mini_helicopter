#ifndef __sand_data_H
#define __sand_data_H	 
#include "sys.h"


void sand_2402_data(s16 a_off_x,s16 a_off_y,s16 a_off_z,s16 g_off_x,s16 g_off_y,s16 g_off_z);
void sand_ACC_GYRO_data(void);
void sand_IMU_data(void);
void sand_RC_data(void);
void sand_Motor_data(void);
void sand_PID_shell_data(void);
void sand_PID_core_data(void);



void Data_Receive_Prepare(u8 data);
void Data_Receive_Anl(u8 *data_buf,u8 num);

#endif





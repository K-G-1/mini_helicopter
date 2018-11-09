#ifndef __param_H
#define __param_H
#include "stm32f0xx.h"
	

/***************************************************/
#define PARAM_ADDR            0X0800F000
#define save_gyro_x           PARAM_ADDR + 0x00
#define save_gyro_y           PARAM_ADDR + 0x02
#define save_gyro_z           PARAM_ADDR + 0x04
 
#define save_acc_x            PARAM_ADDR + 0x06
#define save_acc_y            PARAM_ADDR + 0x08
#define save_acc_z            PARAM_ADDR + 0x0A



#define Shell_Pitch_P         PARAM_ADDR + 0x20
#define Shell_Pitch_I         PARAM_ADDR + 0x22
#define Shell_Pitch_D         PARAM_ADDR + 0x24

#define Shell_Roll_P          PARAM_ADDR + 0x28
#define Shell_Roll_I          PARAM_ADDR + 0x2a
#define Shell_Roll_D          PARAM_ADDR + 0x2c

#define Shell_Yaw_P           PARAM_ADDR + 0x2e
#define Shell_Yaw_I           PARAM_ADDR + 0x30
#define Shell_Yaw_D           PARAM_ADDR + 0x32

#define Core_Pitch_P          PARAM_ADDR + 0x34
#define Core_Pitch_I          PARAM_ADDR + 0x36
#define Core_Pitch_D          PARAM_ADDR + 0x38

#define Core_Roll_P           PARAM_ADDR + 0x3a
#define Core_Roll_I           PARAM_ADDR + 0x3c
#define Core_Roll_D           PARAM_ADDR + 0x3e

#define Core_Yaw_P            PARAM_ADDR + 0x40
#define Core_Yaw_I            PARAM_ADDR + 0x42
#define Core_Yaw_D            PARAM_ADDR + 0x44



#define Hight_P         PARAM_ADDR + 0x60
#define Hight_I         PARAM_ADDR + 0x62
#define Hight_D         PARAM_ADDR + 0x64




void Save_Acc_Gyro_offest(void);
void Save_PID_shell(void);
void Save_PID_core(void);
void Save_PID_hight(void);


void Read_Acc_Gyro_offest(void);
void Read_PID_shell(void);
void Read_PID_core(void);
void Read_PID_hight(void);

#endif

















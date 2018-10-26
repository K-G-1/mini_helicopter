#include "param.h" 
#include "mpu6050.h"
#include "control.h"
#include "stmflash.h"


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))


/******************************Êý¾Ý´¢´æ*************************************************/





void Save_Acc_Gyro_offest(void)
{
    STMFLASH_Write(save_gyro_x,(uint16_t*)sensor.gyro.quiet.x,1);
	STMFLASH_Write(save_gyro_y,(uint16_t*)sensor.gyro.quiet.y,1);
    STMFLASH_Write(save_gyro_z,(uint16_t*)sensor.gyro.quiet.z,1);
    
    STMFLASH_Write(save_acc_x,(uint16_t*)sensor.acc.quiet.x,1);
	STMFLASH_Write(save_acc_y,(uint16_t*)sensor.acc.quiet.y,1);
    STMFLASH_Write(save_acc_z,(uint16_t*)sensor.acc.quiet.z,1);
}





void Save_PID_shell(void)
{
	uint16_t temp_P,temp_I,temp_D;
	
	temp_P= ctrl.pitch.shell.kp *1000;
	temp_I= ctrl.pitch.shell.ki *1000;
	temp_D= ctrl.pitch.shell.kd *1000;
	
    STMFLASH_Write(Shell_Pitch_P,&temp_P,1);
	STMFLASH_Write(Shell_Pitch_I,&temp_I,1);
    STMFLASH_Write(Shell_Pitch_D,&temp_D,1);

	
	temp_P= ctrl.roll.shell.kp *1000;
	temp_I= ctrl.roll.shell.ki *1000;
	temp_D= ctrl.roll.shell.kd *1000;
	
    STMFLASH_Write(Shell_Roll_P,&temp_P,1);
	STMFLASH_Write(Shell_Roll_I,&temp_I,1);
    STMFLASH_Write(Shell_Roll_D,&temp_D,1);
	
	
	temp_P= ctrl.yaw.shell.kp *1000;
	temp_I= ctrl.yaw.shell.ki *1000;
	temp_D= ctrl.yaw.shell.kd *1000;
	
    STMFLASH_Write(Shell_Yaw_P,&temp_P,1);
	STMFLASH_Write(Shell_Yaw_I,&temp_I,1);
    STMFLASH_Write(Shell_Yaw_D,&temp_D,1);	
	
}



void Save_PID_core(void)
{
	uint16_t temp_P,temp_I,temp_D;
	
	temp_P= ctrl.pitch.core.kp *1000;
	temp_I= ctrl.pitch.core.ki *1000;
	temp_D= ctrl.pitch.core.kd *1000;
	
    STMFLASH_Write(Core_Pitch_P,&temp_P,1);
	STMFLASH_Write(Core_Pitch_I,&temp_I,1);
    STMFLASH_Write(Core_Pitch_D,&temp_D,1);
	
	
	temp_P= ctrl.roll.core.kp *1000;
	temp_I= ctrl.roll.core.ki *1000;
	temp_D= ctrl.roll.core.kd *1000;
	
    STMFLASH_Write(Core_Roll_P,&temp_P,1);
	STMFLASH_Write(Core_Roll_I,&temp_I,1);
    STMFLASH_Write(Core_Roll_D,&temp_D,1);
	
	
	temp_P= ctrl.yaw.core.kp *1000;
	temp_I= ctrl.yaw.core.ki *1000;
	temp_D= ctrl.yaw.core.kd *1000;
	
    STMFLASH_Write(Core_Yaw_P,&temp_P,1);
	STMFLASH_Write(Core_Yaw_I,&temp_I,1);
    STMFLASH_Write(Core_Yaw_D,&temp_D,1);
}


void Save_PID_hight(void)
{
	uint16_t temp_P,temp_I,temp_D;
	
	temp_P= ctrl.height.shell.kp *1000;
	temp_I= ctrl.height.shell.ki *1000;
	temp_D= ctrl.height.shell.kd *1000;

    STMFLASH_Write(Hight_P,&temp_P,1);
	STMFLASH_Write(Hight_I,&temp_I,1);
    STMFLASH_Write(Hight_D,&temp_D,1);

}

/********************/


void Read_Acc_Gyro_offest(void)
{
    STMFLASH_Read(save_gyro_x,(uint16_t*)sensor.gyro.quiet.x,1);
	STMFLASH_Read(save_gyro_y,(uint16_t*)sensor.gyro.quiet.y,1);
    STMFLASH_Read(save_gyro_z,(uint16_t*)sensor.gyro.quiet.z,1);
    
    STMFLASH_Read(save_acc_x,(uint16_t*)sensor.acc.quiet.x,1);
	STMFLASH_Read(save_acc_y,(uint16_t*)sensor.acc.quiet.y,1);
    STMFLASH_Read(save_acc_z,(uint16_t*)sensor.acc.quiet.z,1);

}


void Read_PID_shell(void)
{
	uint16_t temp_P,temp_I,temp_D;
	
	STMFLASH_Read(Shell_Pitch_P,&temp_P,1);
	STMFLASH_Read(Shell_Pitch_I,&temp_I,1);
    STMFLASH_Read(Shell_Pitch_D,&temp_D,1);
	
	ctrl.pitch.shell.kp= (float)temp_P/1000.0f;
	ctrl.pitch.shell.ki= (float)temp_I/1000.0f;
	ctrl.pitch.shell.kd= (float)temp_D/1000.0f;
	
	
	STMFLASH_Read(Shell_Roll_P,&temp_P,1);
	STMFLASH_Read(Shell_Roll_I,&temp_I,1);
    STMFLASH_Read(Shell_Roll_D,&temp_D,1);
	
	ctrl.roll.shell.kp= (float)temp_P/1000.0f;
	ctrl.roll.shell.ki= (float)temp_I/1000.0f;
	ctrl.roll.shell.kd= (float)temp_D/1000.0f;
	
	
	STMFLASH_Read(Shell_Yaw_P,&temp_P,1);
	STMFLASH_Read(Shell_Yaw_I,&temp_I,1);
    STMFLASH_Read(Shell_Yaw_D,&temp_D,1);
	
	ctrl.yaw.shell.kp= (float)temp_P/1000.0f;
	ctrl.yaw.shell.ki= (float)temp_I/1000.0f;
	ctrl.yaw.shell.kd= (float)temp_D/1000.0f;
	
}




void Read_PID_core(void)
{
		uint16_t temp_P,temp_I,temp_D;
	
    STMFLASH_Read(Core_Pitch_P,&temp_P,1);
	STMFLASH_Read(Core_Pitch_I,&temp_I,1);
    STMFLASH_Read(Core_Pitch_D,&temp_D,1);	
    
	ctrl.pitch.core.kp= (float)temp_P/1000.0f;
	ctrl.pitch.core.ki= (float)temp_I/1000.0f;
	ctrl.pitch.core.kd= (float)temp_D/1000.0f;
	
	
    STMFLASH_Read(Core_Roll_P,&temp_P,1);
	STMFLASH_Read(Core_Roll_I,&temp_I,1);
    STMFLASH_Read(Core_Roll_D,&temp_D,1);	
	
	ctrl.roll.core.kp= (float)temp_P/1000.0f;
	ctrl.roll.core.ki= (float)temp_I/1000.0f;
	ctrl.roll.core.kd= (float)temp_D/1000.0f;
	
    STMFLASH_Read(Core_Yaw_P,&temp_P,1);
	STMFLASH_Read(Core_Yaw_I,&temp_I,1);
    STMFLASH_Read(Core_Yaw_D,&temp_D,1);	
	
	ctrl.yaw.core.kp= (float)temp_P/1000.0f;
	ctrl.yaw.core.ki= (float)temp_I/1000.0f;
	ctrl.yaw.core.kd= (float)temp_D/1000.0f;
	
}

void Read_PID_hight(void)
{
		uint16_t temp_P,temp_I,temp_D;
	
    STMFLASH_Read(Hight_P,&temp_P,1);
	STMFLASH_Read(Hight_I,&temp_I,1);
    STMFLASH_Read(Hight_D,&temp_D,1);	
	
	ctrl.height.shell.kp= (float)temp_P/1000.0f;
	ctrl.height.shell.ki= (float)temp_I/1000.0f;
	ctrl.height.shell.kd= (float)temp_D/1000.0f;
	

	
}




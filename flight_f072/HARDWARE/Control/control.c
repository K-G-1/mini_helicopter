#include "control.h"
#include "RC.h"
#include "mpu9250.h"
#include "pwm.h"


struct _ctrl ctrl;


uint16_t Moto_duty[4];
float	Yawtemp2;
float Yaw_offest_tem;
extern float Yaw_offest ;
int date_THROTTLE;
float roll_old,pitch_old;
/***************************************************/
/*void Control(float rol, float pit, float yaw)    */
/*输入：rol   横滚角                               */
/*      pit   俯仰角                               */
/*		  yaw   航向角                               */
/*输出：                                           */
/*备注：串级PID 控制   外环（角度环）采用PID调节   */
/*                     内环（角速度环）采用PD调节  */
/***************************************************/
void PID_Param_init(void)
{
    ctrl.pitch.shell.kp = 10;
    ctrl.pitch.shell.ki = 0;
    ctrl.pitch.shell.kd = 0;
    ctrl.pitch.core.kp = 10;
    ctrl.pitch.core.ki = 0.00;
    ctrl.pitch.core.kd = 0;
    
    ctrl.roll.shell.kp = 10;
    ctrl.roll.shell.ki = 0.0;
    ctrl.roll.shell.kd = 0;
    ctrl.roll.core.kp = 10;
    ctrl.roll.core.ki = 0.0;
    ctrl.roll.core.kd = 0;
    
    ctrl.yaw.shell.kp = 10;
    ctrl.yaw.shell.ki = 0.0;
    ctrl.yaw.shell.kd = 0;
    ctrl.yaw.core.kp = 10;
    ctrl.yaw.core.ki = 0.0;
    ctrl.yaw.core.kd = 0;

    
    ctrl.pitch.shell.increment_max = 100;
    ctrl.pitch.core.increment_max = 100;
    
    ctrl.roll.shell.increment_max = 100;
    ctrl.roll.core.increment_max = 100;

}
void CONTROL(float rol, float pit, float yaw)
{
	static float roll_old,pitch_old;
//	float Yawtemp1;
	
	
 
	if(ctrl.ctrlRate >= 2)  //内环进行2次控制   外环进行1次控制   内环控制频率为外环的2倍 
	{
		//*****************外环PID**************************//
		//俯仰计算//
		pit= pit + (float)(Rc_Data.PITCH - Rc_Data.pitch_offset)/20.0f;
		ctrl.pitch.shell.increment += pit;   //俯仰方向误差积分
			
			//积分限幅
		if(ctrl.pitch.shell.increment > ctrl.pitch.shell.increment_max)
				ctrl.pitch.shell.increment = ctrl.pitch.shell.increment_max;
		else if(ctrl.pitch.shell.increment < -ctrl.pitch.shell.increment_max)
				ctrl.pitch.shell.increment = -ctrl.pitch.shell.increment_max; 
		
		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * pit + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment + ctrl.pitch.shell.kd * (pit - pitch_old);
		pitch_old = pit; //储存 俯仰偏差
		
		//横滚计算//
		rol= rol + (float)(Rc_Data.ROLL - Rc_Data.roll_offset)/20.0f;
		ctrl.roll.shell.increment += rol;  //横滚方向误差积分
			
			//积分限幅
		if(ctrl.roll.shell.increment > ctrl.roll.shell.increment_max)
				ctrl.roll.shell.increment = ctrl.roll.shell.increment_max;
		else if(ctrl.roll.shell.increment < -ctrl.roll.shell.increment_max)
				ctrl.roll.shell.increment = -ctrl.roll.shell.increment_max;	 

		ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * rol + ctrl.roll.shell.ki * ctrl.roll.shell.increment + ctrl.roll.shell.kd * (rol - roll_old);
		roll_old = rol;  //储存 横滚偏差

		//航向计算////////////
		
		ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp * (Rc_Data.YAW - Rc_Data.yaw_offset)/5 + ctrl.yaw.shell.kd * sensor.gyro.origin.z;	
		
//		Yaw_offest_tem=Yaw_offest + (Rc_Data.YAW - Rc_Data.yaw_offset)/20 ;
//		if(Yaw_offest_tem<0)
//			Yaw_offest_tem=Yaw_offest_tem+360;
//		else if(Yaw_offest_tem>360)
//			Yaw_offest_tem=Yaw_offest_tem-360;
//    else Yaw_offest_tem=Yaw_offest_tem;
//			
//			
//		Yawtemp1=Yaw_offest_tem-yaw;
//		if(yaw<Yaw_offest_tem)
//		Yawtemp1=360-Yaw_offest_tem+yaw;		
//		
//			Yawtemp2=Yawtemp1;
//    if(Yawtemp1>180)
//		 Yawtemp2=Yawtemp1-360;
//		
//		ctrl.yaw.shell.increment += (-Yawtemp2);
//		//积分限幅    
//		if(ctrl.yaw.shell.increment > ctrl.yaw.shell.increment_max)
//				ctrl.yaw.shell.increment = ctrl.yaw.shell.increment_max;
//		else if(ctrl.yaw.shell.increment < -ctrl.yaw.shell.increment_max)
//				ctrl.yaw.shell.increment = -ctrl.yaw.shell.increment_max;
//		  
//		
//    ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp * (-Yawtemp2) +ctrl.yaw.shell.ki * ctrl.yaw.shell.increment+ ctrl.yaw.shell.kd * (yaw - yaw_old);		
		ctrl.ctrlRate = 0;
	}
	ctrl.ctrlRate ++;	
	//********************内环(角速度环)PID*********************************//

	ctrl.roll.core.kp_out = ctrl.roll.core.kp * (ctrl.roll.shell.pid_out + sensor.gyro.averag.x ); 
	ctrl.roll.core.increment += (ctrl.roll.shell.pid_out+ sensor.gyro.averag.x  ) ;
			//积分限幅
		if(ctrl.roll.core.increment > ctrl.roll.core.increment_max)
				ctrl.roll.core.increment = ctrl.roll.core.increment_max;
		else if(ctrl.roll.core.increment < -ctrl.roll.core.increment_max)
				ctrl.roll.core.increment = -ctrl.roll.core.increment_max; 
	ctrl.roll.core.ki_out	= ctrl.roll.core.ki * ctrl.roll.core.increment ;
	ctrl.roll.core.kd_out = ctrl.roll.core.kd * (sensor.gyro.averag.x  - sensor.gyro.histor.x);
	
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * (ctrl.pitch.shell.pid_out+ sensor.gyro.averag.y  ) ;
	ctrl.pitch.core.increment += (ctrl.pitch.shell.pid_out+ sensor.gyro.averag.y  ) ;
			//积分限幅
		if(ctrl.pitch.core.increment > ctrl.pitch.core.increment_max)
				ctrl.pitch.core.increment = ctrl.pitch.core.increment_max;
		else if(ctrl.pitch.core.increment < -ctrl.pitch.core.increment_max)
				ctrl.pitch.core.increment = -ctrl.pitch.core.increment_max; 	
	ctrl.pitch.core.ki_out	= ctrl.pitch.core.ki * ctrl.pitch.core.increment ;
	ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (sensor.gyro.averag.y - sensor.gyro.histor.y);
	
	ctrl.yaw.core.kp_out = ctrl.yaw.core.kp * (ctrl.yaw.shell.pid_out + sensor.gyro.averag.z );
	ctrl.yaw.core.kd_out = ctrl.yaw.core.kd * (sensor.gyro.averag.z - sensor.gyro.histor.z);
	
	ctrl.roll.core.pid_out = ctrl.roll.core.kp_out + ctrl.roll.core.ki_out + ctrl.roll.core.kd_out;
	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.ki_out + ctrl.pitch.core.kd_out;
	ctrl.yaw.core.pid_out =  ctrl.yaw.core.kp_out + ctrl.yaw.core.kd_out;	 
	
	if(ctrl.yaw.core.pid_out > 100)
		ctrl.yaw.core.pid_out = 100;
	if(ctrl.yaw.core.pid_out < -100)
		ctrl.yaw.core.pid_out = -100;
	
	sensor.gyro.histor.x = sensor.gyro.averag.x;   //储存陀螺仪（角速度）历史值
	sensor.gyro.histor.y = sensor.gyro.averag.y;
	sensor.gyro.histor.z = sensor.gyro.averag.z;
	
//	ctrl.roll.core.pid_out = ctrl.roll.core.kp_out+ctrl.roll.core.ki_out + ctrl.roll.core.kd_out;
//	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out +ctrl.pitch.core.ki_out+ ctrl.pitch.core.kd_out;
//	ctrl.yaw.core.pid_out =  ctrl.yaw.core.kp_out+ctrl.yaw.core.ki_out + ctrl.yaw.core.kd_out;

//	ctrl.pitch.core.pid_out = ctrl.pitch.core.pid_out*0.8 + ctrl.pitch.shell.pid_out/2;//经验值，输出最稳定的调试比例
//	ctrl.roll.core.pid_out  = ctrl.roll.core.pid_out *0.8 + ctrl.roll.shell.pid_out/2; 
//	ctrl.yaw.core.pid_out   = ctrl.yaw.core.pid_out ;

	
//	ctrl.roll.core.pid_out = ctrl.roll.core.kp_out;
//	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out;
//	ctrl.yaw.core.pid_out =  ctrl.yaw.core.kp_out ;
	
	
	

/*         控制采用X模式          */
/*           2     1              */
/*            \   /               */ 
/*             \ /                */
/*             / \                */
/*            /   \               */
/*           3     4              */
/* 1:Moto_duty[0]  2:Moto_duty[1] */
/* 3:Moto_duty[2]  4:Moto_duty[3] */	
	if( Rc_Data.THROTTLE > 1050)	
	{	
		
		if(mode ==1||mode== 0 )
		{
				date_THROTTLE	= Rc_Data.THROTTLE -1000;///cos(angle.roll/57.3)/cos(angle.pitch/57.3);  //油门倾角补偿，防止因倾斜高度下降太快		
				Moto_duty[0] = date_THROTTLE  + ctrl.yaw.core.pid_out - ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out; 
				Moto_duty[1] = date_THROTTLE  - ctrl.yaw.core.pid_out + ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out;
				Moto_duty[2] = date_THROTTLE  + ctrl.yaw.core.pid_out + ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out;
				Moto_duty[3] = date_THROTTLE  - ctrl.yaw.core.pid_out - ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out; 	
		}
		else if(mode ==2)
		{
				Moto_duty[0] = date_THROTTLE + ctrl.height.shell.pid_out + ctrl.yaw.core.pid_out - ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out; 
				Moto_duty[1] = date_THROTTLE + ctrl.height.shell.pid_out - ctrl.yaw.core.pid_out + ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out;
				Moto_duty[2] = date_THROTTLE + ctrl.height.shell.pid_out + ctrl.yaw.core.pid_out + ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out;
				Moto_duty[3] = date_THROTTLE + ctrl.height.shell.pid_out - ctrl.yaw.core.pid_out - ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out; 			
//			if(ctrl.height.shell.pid_out<250&&g_Alt_Hight<110)
//			{Moto_duty[0] = Moto_duty[1] = Moto_duty[2] = Moto_duty[3] = 0;}	
//					Moto_duty[0] = date_THROTTLE + ctrl.height.shell.pid_out;
		}

		
//		Moto_duty[0] = ctrl.height.shell.pid_out -1000  - ctrl.yaw.core.pid_out;
//		Moto_duty[1] = date_THROTTLE -1000  + ctrl.yaw.core.pid_out; 
// 		Moto_duty[2] = date_THROTTLE -1000  - ctrl.yaw.core.pid_out; 
//		Moto_duty[3] = date_THROTTLE -1000  + ctrl.yaw.core.pid_out; 
//		
	}
	else 
	{
		Moto_duty[0] = Moto_duty[1] = Moto_duty[2] = Moto_duty[3] = 0;
		ctrl.pitch.shell.increment = 0;
		ctrl.roll.shell.increment = 0;
		ctrl.pitch.core.increment = 0;
		ctrl.roll.core.increment = 0;
	}
	
	if(Moto_duty[0]<=0) Moto_duty[0] = 0;
	if(Moto_duty[1]<=0) Moto_duty[1] = 0;
	if(Moto_duty[2]<=0) Moto_duty[2] = 0;
	if(Moto_duty[3]<=0) Moto_duty[3] = 0;
	
	if(ARMED)
    
		Moto_PwmRflash(Moto_duty[0],Moto_duty[1],Moto_duty[2],Moto_duty[3]);
	else 
	{

		Moto_PwmRflash(0,0,0,0);
	}
}

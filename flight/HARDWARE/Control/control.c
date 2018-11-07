#include "control.h"
#include "RC.h"
#include "mpu9250.h"
#include "pwm.h"
#include "Algorithm_math.h"

struct _ctrl ctrl;


vs16 Moto_duty[4];
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
  
  //roll方向的角度环PID
	ctrl.roll.shell.Exp = (float)((Rc_Data.ROLL - 1500 )/12.0f);                                                                      //期望
  ctrl.roll.shell.error = ctrl.roll.shell.Exp + rol;                                                                                //误差
  ctrl.roll.shell.increment += ctrl.roll.shell.error;                                                                               //积分
  ctrl.roll.shell.differ = ctrl.roll.shell.error - ctrl.roll.shell.PreErr;                                                          //微分
  
  ctrl.roll.shell.increment = data_limit(ctrl.roll.shell.increment,ctrl.roll.shell.increment_max,-ctrl.roll.shell.increment_max);   //积分限幅
  ctrl.roll.shell.PreErr = ctrl.roll.shell.error;
  
  ctrl.roll.shell.kp_out = ctrl.roll.shell.kp * ctrl.roll.shell.error;
  ctrl.roll.shell.ki_out = ctrl.roll.shell.ki * ctrl.roll.shell.increment;
  ctrl.roll.shell.kd_out = ctrl.roll.shell.kd * ctrl.roll.shell.differ;
  
  ctrl.roll.shell.pid_out = ctrl.roll.shell.kp_out + ctrl.roll.shell.ki_out + ctrl.roll.shell.kd_out;

  //pitch方向的角度环PID
	ctrl.pitch.shell.Exp = (float)((Rc_Data.PITCH - 1500 )/12.0f);                                                                      //期望
  ctrl.pitch.shell.error = ctrl.pitch.shell.Exp - pit;                                                                                //误差
  ctrl.pitch.shell.increment += ctrl.pitch.shell.error;                                                                               //积分
  ctrl.pitch.shell.differ = ctrl.pitch.shell.error - ctrl.pitch.shell.PreErr;                                                          //微分
  
  ctrl.pitch.shell.increment = data_limit(ctrl.pitch.shell.increment,ctrl.pitch.shell.increment_max,-ctrl.pitch.shell.increment_max);   //积分限幅
  ctrl.pitch.shell.PreErr = ctrl.pitch.shell.error;
  
  ctrl.pitch.shell.kp_out = ctrl.pitch.shell.kp * ctrl.pitch.shell.error;
  ctrl.pitch.shell.ki_out = ctrl.pitch.shell.ki * ctrl.pitch.shell.increment;
  ctrl.pitch.shell.kd_out = ctrl.pitch.shell.kd * ctrl.pitch.shell.differ;
  
  ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp_out + ctrl.pitch.shell.ki_out + ctrl.pitch.shell.kd_out;
  
  //
  ctrl.yaw.shell.Exp = (float)((Rc_Data.YAW - 1500 )/12.0f);                                                                      //期望
  ctrl.yaw.shell.error = ctrl.yaw.shell.Exp ;                                                                                     //误差
  ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp * ctrl.yaw.shell.error;
 /************************************************/ 
  //roll方向的角速度环PID
  ctrl.roll.core.Exp = ctrl.roll.shell.pid_out;                                                                                   //期望
  ctrl.roll.core.error = ctrl.roll.core.Exp + (sensor.gyro.radian.x * RtA);                                                       //误差
  ctrl.roll.core.increment += ctrl.roll.core.error;                                                                               //积分
  ctrl.roll.core.differ = ctrl.roll.core.error - ctrl.roll.core.PreErr;                                                           //微分
  
  ctrl.roll.core.increment = data_limit(ctrl.roll.core.increment,ctrl.roll.core.increment_max,-ctrl.roll.core.increment_max);     //积分限幅
  ctrl.roll.core.PreErr = ctrl.roll.core.error;
  
  ctrl.roll.core.kp_out = ctrl.roll.core.kp * ctrl.roll.core.error;
  ctrl.roll.core.ki_out = ctrl.roll.core.ki * ctrl.roll.core.increment;
  ctrl.roll.core.kd_out = ctrl.roll.core.kd * ctrl.roll.core.differ;
  
  ctrl.roll.core.pid_out = ctrl.roll.core.kp_out + ctrl.roll.core.ki_out + ctrl.roll.core.kd_out;
  
  //pitch方向的角速度度环PID
  ctrl.pitch.core.Exp = ctrl.pitch.shell.pid_out;                                                                                   //期望
  ctrl.pitch.core.error = ctrl.pitch.core.Exp + (sensor.gyro.radian.y * RtA);                                                       //误差
  ctrl.pitch.core.increment += ctrl.pitch.core.error;                                                                               //积分
  ctrl.pitch.core.differ = ctrl.pitch.core.error - ctrl.pitch.core.PreErr;                                                           //微分
  
  ctrl.pitch.core.increment = data_limit(ctrl.pitch.core.increment,ctrl.pitch.core.increment_max,-ctrl.pitch.core.increment_max);     //积分限幅
  ctrl.pitch.core.PreErr = ctrl.pitch.core.error;
  
  ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * ctrl.pitch.core.error;
  ctrl.pitch.core.ki_out = ctrl.pitch.core.ki * ctrl.pitch.core.increment;
  ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * ctrl.pitch.core.differ;
  
  ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.ki_out + ctrl.pitch.core.kd_out;
  
  //YAW的角速度环，与上面有些不同，因为没有磁力计
  ctrl.yaw.core.Exp = ctrl.yaw.shell.pid_out ;                                                                                   //期望
  ctrl.yaw.core.error = ctrl.yaw.core.Exp + (sensor.gyro.radian.z * RtA);                                                       //误差
  ctrl.yaw.core.increment += ctrl.yaw.core.error;                                                                               //积分
  ctrl.yaw.core.differ = ctrl.yaw.core.error - ctrl.yaw.core.PreErr;                                                           //微分
  
  ctrl.yaw.core.increment = data_limit(ctrl.yaw.core.increment,ctrl.yaw.core.increment_max,-ctrl.yaw.core.increment_max);     //积分限幅
  ctrl.yaw.core.PreErr = ctrl.yaw.core.error;
  
  ctrl.yaw.core.kp_out = ctrl.yaw.core.kp * ctrl.yaw.core.error;
  ctrl.yaw.core.ki_out = ctrl.yaw.core.ki * ctrl.yaw.core.increment;
  ctrl.yaw.core.kd_out = ctrl.yaw.core.kd * ctrl.yaw.core.differ;
  
  ctrl.yaw.core.pid_out = ctrl.yaw.core.kp_out + ctrl.yaw.core.ki_out + ctrl.yaw.core.kd_out;
  
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
      
//        Moto_duty[0] = date_THROTTLE  + ctrl.yaw.shell.pid_out ; 
//				Moto_duty[1] = date_THROTTLE  - ctrl.yaw.shell.pid_out ;
//				Moto_duty[2] = date_THROTTLE  + ctrl.yaw.shell.pid_out ;
//				Moto_duty[3] = date_THROTTLE  - ctrl.yaw.shell.pid_out ;
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
    
		Moto_PwmRflash(Moto_duty[0] * 4,Moto_duty[1]*4,Moto_duty[2]*4,Moto_duty[3]*4);
	else 
	{

		Moto_PwmRflash(0,0,0,0);
	}
//	Moto_duty[0] = Moto_duty[1] = Moto_duty[2] = Moto_duty[3] = date_THROTTLE;
//	Moto_PwmRflash(date_THROTTLE,date_THROTTLE,date_THROTTLE,date_THROTTLE);	
}

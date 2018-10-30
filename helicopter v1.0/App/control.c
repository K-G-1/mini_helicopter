
#include "control.h"
#include "mpu6050.h"
#include "Imu.h"
#include "PWM_output.h"
#include "param.h"
#include "mpu6050.h"
#define AnglePwmMax 500
#define SpeedPwmMax 500
#define DirectionPwmMax 199
#define DeadPwm 0
#define MotorPwmMax 1199

#define IntegralExternal	300
#define IntegralInternal	300

#define RC_CTRL 8
/****************角度计算和融合相关定义*********************/
#define Gyro_ratio	16.4       // 
#define PI 			3.14159

T_int16_xyz			g_t_AccOffset_CMS;	
T_int16_xyz			g_t_GyroOffset_CMS;			//零漂
T_int16_xyz 		g_t_MpuDataAccel1_CMS;	//MPU6050 Accel data ，读两次平均
T_int16_xyz 		g_t_MpuDataAccel2_CMS;
T_int16_xyz 		g_t_MpuDataGyro1_CMS;	//MPU6050 Gyro data ，读两次平均
T_int16_xyz 		g_t_MpuDataGyro2_CMS;
T_int16_xyz			g_t_Accel_CMS;			//两次平均后的Accel数据
T_int16_xyz			g_t_Gyro_CMS;			//两次平均后的Gyro数据
T_int16_xyz			g_t_AccelAVG_CMS;		//20次环形滤波器滤波后Accel数据
T_float_angle 		g_t_AttAngle_CMS;		//ATT函数计算出的姿态角
vs32				g_vs32_Alt_CMS;
T_RC_Data 			g_t_Rc_D_CMS;			//遥控通道数据
T_RC_Control		g_t_Rc_C_CMS;			//遥控功能数据
T_PID  				g_t_PidRol_CMS;			//Rol pid data
T_PID  				g_t_PidPit_CMS;			//Pit pid data
T_PID  				g_t_PidYaw_CMS;			//Yaw pid data

float g_fl_RolIntegral_CMS = 0;	//ROL积分
float g_fl_PitIntegral_CMS = 0;	//PIT积分
float g_fl_YawProportion_CMS = 0;	//YAW比例

float g_fl_RolIntegralExternal_CMS = 0;		
float g_fl_PitIntegralExternal_CMS = 0;
float g_fl_YawProportionExternal_CMS = 0;

float g_fl_RolIntegralInternal_CMS = 0;		
float g_fl_PitIntegralInternal_CMS = 0;
float g_fl_YawProportionInternal_CMS = 0;


int16_t temp_rolpwmP,temp_rolpwmI,temp_pitpwmP,temp_pitpwmI;


void g_v_PIDInit(void)
{
//	g_t_PidRol_CMS.P = 0.05;
//	g_t_PidRol_CMS.I = 0.001;
//	g_t_PidRol_CMS.D = 0.001;
//	

//	g_t_PidPit_CMS.I = 0.000;
//	g_t_PidPit_CMS.D = 0.000;	
//	
//	g_t_PidYaw_CMS.P = 0.00;
//	g_t_PidYaw_CMS.I = 0.000;
//	g_t_PidYaw_CMS.D = 0.000;	
		
	g_t_Rc_D_CMS.THROTTLE = 1000;
	g_t_Rc_C_CMS.ARMED = 0;
/***********单环yaw*******************/
	g_t_PidYaw_CMS.P = ANO_Param.PID_yaw_s.kp*0.01;
	g_t_PidYaw_CMS.D = ANO_Param.PID_yaw_s.kd*0.001;
///******************双环rol pit pid参数***********************/
	g_t_PidRol_CMS.InternalP = ANO_Param.PID_rol_s.kp*0.001;//1;
	g_t_PidRol_CMS.InternalI = ANO_Param.PID_rol_s.ki*0.001;//0.04;
	g_t_PidRol_CMS.InternalD = ANO_Param.PID_rol_s.kd*0.001;//0.2;
	
	g_t_PidRol_CMS.ExternalP = ANO_Param.PID_rol.kp*0.01;//1.5;
	g_t_PidRol_CMS.ExternalI = ANO_Param.PID_rol.ki*0.001;//0.005;
	
	g_t_PidPit_CMS.InternalP = ANO_Param.PID_pit_s.kp*0.001f;//1.0;
	g_t_PidPit_CMS.InternalI = ANO_Param.PID_pit_s.ki*0.001f;//0.04;
	g_t_PidPit_CMS.InternalD = ANO_Param.PID_pit_s.kd*0.001f;//0.2;

	g_t_PidPit_CMS.ExternalP = ANO_Param.PID_pit.kp*0.01f;//1.5;
	g_t_PidPit_CMS.ExternalI = ANO_Param.PID_pit.ki*0.001f;//0.005;	
/*     王祥pid2参数            */
//		g_t_PidRol_CMS.InternalP = 0.78;//1;
//		g_t_PidRol_CMS.InternalI = 0.001;//0.04;
//		g_t_PidRol_CMS.InternalD = 4.0;//0.2;
//	
//		g_t_PidRol_CMS.ExternalP = 15.5;//1.5;
//		g_t_PidRol_CMS.ExternalI = 0.001;//0.005;
//	
//		g_t_PidPit_CMS.InternalP = 0.78;//1.0;
//		g_t_PidPit_CMS.InternalI = 0.001;//0.04;
//		g_t_PidPit_CMS.InternalD = 4.0;//0.2;
//	
//		g_t_PidPit_CMS.ExternalP = 15.5;//1.5;
//		g_t_PidPit_CMS.ExternalI = 0.001;//0.005;
//	
//		g_t_PidYaw_CMS.P = 0.2;
//		g_t_PidYaw_CMS.D = 0.2;
}

/****************************end****************************/

void g_v_GetMpuOffset(void)
{
	T_int16_xyz			g_t_AccOffsetTemp_CMS;
	T_int16_xyz			g_t_GyroOffsetTemp_CMS;	
	static int32_t	tempgx=0u,tempgy=0u,tempgz=0u;
	static int32_t	tempax=0u,tempay=0u,tempaz=0u;
	static uint8_t cnt=0u;
	static uint8_t offsetOK=0u;
	if(offsetOK == 0u)
	{

		mpu6050_read_reg(ACCEL_XOUT_H,0,14);
		//读取XYZ轴加速度计的值
		g_t_AccOffsetTemp_CMS.X = (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_XOUT_H_buf_add] * 256 
								+ (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_XOUT_L_buf_add] ;
		g_t_AccOffsetTemp_CMS.Y = (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_YOUT_H_buf_add] * 256 
								+ (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_YOUT_L_buf_add] ;
		g_t_AccOffsetTemp_CMS.Z = (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_ZOUT_H_buf_add] * 256 
								+ (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_ZOUT_L_buf_add] ;
			
		//读取XYZ轴陀螺仪的值
		g_t_GyroOffsetTemp_CMS.X = (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_XOUT_H_buf_add] * 256 
								+ (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_XOUT_L_buf_add] ; 
		g_t_GyroOffsetTemp_CMS.Y = (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_YOUT_H_buf_add] * 256 
								+ (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_YOUT_L_buf_add] ; 
		g_t_GyroOffsetTemp_CMS.Z = (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_ZOUT_H_buf_add] * 256 
								+ (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_ZOUT_L_buf_add] ; 
		
		tempax += g_t_AccOffsetTemp_CMS.X;
		tempay += g_t_AccOffsetTemp_CMS.Y;
		tempaz += g_t_AccOffsetTemp_CMS.Z;
		tempgx += g_t_GyroOffsetTemp_CMS.X;
		tempgy += g_t_GyroOffsetTemp_CMS.Y;
		tempgz += g_t_GyroOffsetTemp_CMS.Z;

		cnt++;
		if(cnt == 200u)
		{
			offsetOK = 1u;
			g_t_AccOffset_CMS.X = tempax * 1.0/ cnt;
			g_t_AccOffset_CMS.Y = tempay * 1.0/ cnt;
			g_t_AccOffset_CMS.Z = tempaz * 1.0/ cnt;
			
			g_t_GyroOffset_CMS.X = tempgx * 1.0/ cnt;
			g_t_GyroOffset_CMS.Y = tempgy * 1.0/ cnt;
			g_t_GyroOffset_CMS.Z = tempgz * 1.0/ cnt;
			cnt = 0u;
			tempax = 0;
			tempay = 0;
			tempaz = 0;
			tempgx = 0;
			tempgy = 0;
			tempgz = 0;
		}
	}	
}
#define 	MPU6050_MAX		32767
#define		MPU6050_MIN		-32768
void g_v_GetMpuValue(void)
{
	//读取mpu6050数据，连续读14个字节，存放在mpu_reg_buf中
	mpu6050_read_reg(ACCEL_XOUT_H,0,14);
	
	//读取XYZ轴加速度计的值
	g_t_MpuDataAccel1_CMS.X = (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_XOUT_H_buf_add] * 256 
							+ (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_XOUT_L_buf_add] - ANO_Param.acc_offset.x;
	g_t_MpuDataAccel1_CMS.Y = (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_YOUT_H_buf_add] * 256 
							+ (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_YOUT_L_buf_add] - ANO_Param.acc_offset.y;
	g_t_MpuDataAccel1_CMS.Z = (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_ZOUT_H_buf_add] * 256 
							+ (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_ZOUT_L_buf_add] - ANO_Param.acc_offset.z;
		
	//读取XYZ轴陀螺仪的值
	g_t_MpuDataGyro1_CMS.X = (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_XOUT_H_buf_add] * 256 
						   + (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_XOUT_L_buf_add] - ANO_Param.gyr_offset.x; 
	g_t_MpuDataGyro1_CMS.Y = (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_YOUT_H_buf_add] * 256 
						   + (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_YOUT_L_buf_add] - ANO_Param.gyr_offset.y; 
	g_t_MpuDataGyro1_CMS.Z = (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_ZOUT_H_buf_add] * 256 
						   + (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_ZOUT_L_buf_add] - ANO_Param.gyr_offset.z; 

	mpu6050_read_reg(ACCEL_XOUT_H,0,14);
	//读取XYZ轴加速度计的值
	g_t_MpuDataAccel2_CMS.X = (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_XOUT_H_buf_add] * 256 
							+ (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_XOUT_L_buf_add] - ANO_Param.acc_offset.x;
	g_t_MpuDataAccel2_CMS.Y = (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_YOUT_H_buf_add] * 256 
							+ (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_YOUT_L_buf_add] - ANO_Param.acc_offset.y;
	g_t_MpuDataAccel2_CMS.Z = (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_ZOUT_H_buf_add] * 256 
							+ (int16_t)g_u8a14_MpuRegBuf_SMP[ACCEL_ZOUT_L_buf_add] - ANO_Param.acc_offset.z;
		
	//读取XYZ轴陀螺仪的值
	g_t_MpuDataGyro2_CMS.X = (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_XOUT_H_buf_add] * 256 
						   + (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_XOUT_L_buf_add] - ANO_Param.gyr_offset.x; 
	g_t_MpuDataGyro2_CMS.Y = (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_YOUT_H_buf_add] * 256 
						   + (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_YOUT_L_buf_add] - ANO_Param.gyr_offset.y; 
	g_t_MpuDataGyro2_CMS.Z = (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_ZOUT_H_buf_add] * 256 
						   + (int16_t)g_u8a14_MpuRegBuf_SMP[GYRO_ZOUT_L_buf_add] - ANO_Param.gyr_offset.x; 


//	g_t_MpuDataAccel1_CMS.X>MPU6050_MAX ? g_t_MpuDataAccel1_CMS.X = MPU6050_MAX:g_t_MpuDataAccel1_CMS.X;
//	g_t_MpuDataAccel1_CMS.X<MPU6050_MIN ? g_t_MpuDataAccel1_CMS.X = MPU6050_MIN:g_t_MpuDataAccel1_CMS.X;
//	g_t_MpuDataAccel1_CMS.Y>MPU6050_MAX ? g_t_MpuDataAccel1_CMS.Y = MPU6050_MAX:g_t_MpuDataAccel1_CMS.Y;
//	g_t_MpuDataAccel1_CMS.Y<MPU6050_MIN ? g_t_MpuDataAccel1_CMS.Y = MPU6050_MIN:g_t_MpuDataAccel1_CMS.Y;
//	g_t_MpuDataAccel1_CMS.Z>MPU6050_MAX ? g_t_MpuDataAccel1_CMS.Z = MPU6050_MAX:g_t_MpuDataAccel1_CMS.Z;
//	g_t_MpuDataAccel1_CMS.Z<MPU6050_MIN ? g_t_MpuDataAccel1_CMS.Z = MPU6050_MIN:g_t_MpuDataAccel1_CMS.Z;
//	g_t_MpuDataGyro2_CMS.X >MPU6050_MAX ? g_t_MpuDataGyro2_CMS.X = MPU6050_MAX:g_t_MpuDataGyro2_CMS.X ;
//	g_t_MpuDataGyro2_CMS.X <MPU6050_MIN ? g_t_MpuDataGyro2_CMS.X = MPU6050_MIN:g_t_MpuDataGyro2_CMS.X ;
//	g_t_MpuDataGyro2_CMS.Y>MPU6050_MAX ? g_t_MpuDataGyro2_CMS.Y = MPU6050_MAX:g_t_MpuDataGyro2_CMS.Y;
//	g_t_MpuDataGyro2_CMS.Y<MPU6050_MIN ? g_t_MpuDataGyro2_CMS.Y = MPU6050_MIN:g_t_MpuDataGyro2_CMS.Y;
//	g_t_MpuDataGyro2_CMS.Z>MPU6050_MAX ? g_t_MpuDataGyro2_CMS.Z = MPU6050_MAX:g_t_MpuDataGyro2_CMS.Z;
//	g_t_MpuDataGyro2_CMS.Z<MPU6050_MIN ? g_t_MpuDataGyro2_CMS.Z = MPU6050_MIN:g_t_MpuDataGyro2_CMS.Z;

	//类型转换
//	if(g_t_MpuDataAccel1_CMS.X > 32768)
//		g_t_MpuDataAccel1_CMS.X -= 65535;	
//	if(g_t_MpuDataAccel1_CMS.Y > 32768)
//		g_t_MpuDataAccel1_CMS.Y -= 65535;		
//	if(g_t_MpuDataAccel1_CMS.Z > 32768)
//		g_t_MpuDataAccel1_CMS.Z -= 65535;	

//	if(g_t_MpuDataGyro1_CMS.X > 32768)
//		g_t_MpuDataGyro1_CMS.X -= 65535;	
//	if(g_t_MpuDataGyro1_CMS.Y > 32768)
//		g_t_MpuDataGyro1_CMS.Y -= 65535;
//	if(g_t_MpuDataGyro1_CMS.Z > 32768)
//		g_t_MpuDataGyro1_CMS.Z -= 65535;
//	
//	if(g_t_MpuDataAccel2_CMS.X > 32768)
//		g_t_MpuDataAccel2_CMS.X -= 65535;	
//	if(g_t_MpuDataAccel2_CMS.Y > 32768)
//		g_t_MpuDataAccel2_CMS.Y -= 65535;		
//	if(g_t_MpuDataAccel2_CMS.Z > 32768)
//		g_t_MpuDataAccel2_CMS.Z -= 65535;	

//	if(g_t_MpuDataGyro2_CMS.X > 32768)
//		g_t_MpuDataGyro2_CMS.X -= 65535;	
//	if(g_t_MpuDataGyro2_CMS.Y > 32768)
//		g_t_MpuDataGyro2_CMS.Y -= 65535;
//	if(g_t_MpuDataGyro2_CMS.Z > 32768)
//		g_t_MpuDataGyro2_CMS.Z -= 65535;	
}

//**************************************************************************
/*
*  功能说明:姿态计算
*  参数说明：

*  函数返回：无符号结果值
*  修改时间：2013-2-10
* 备注：参考匿名四轴
*/
void g_v_UpdataPositioning(void)
{	
	g_v_GetMpuValue();
	g_t_Accel_CMS.X = -(g_t_MpuDataAccel1_CMS.Y + g_t_MpuDataAccel2_CMS.Y) / 2;
	g_t_Accel_CMS.Y = (g_t_MpuDataAccel1_CMS.X + g_t_MpuDataAccel2_CMS.X) / 2;
	g_t_Accel_CMS.Z = (g_t_MpuDataAccel1_CMS.Z + g_t_MpuDataAccel2_CMS.Z) / 2;


	
	g_t_Gyro_CMS.X = -(g_t_MpuDataGyro1_CMS.Y + g_t_MpuDataGyro2_CMS.Y) / 2;
	g_t_Gyro_CMS.Y = (g_t_MpuDataGyro1_CMS.X + g_t_MpuDataGyro2_CMS.X) / 2;
	g_t_Gyro_CMS.Z = (g_t_MpuDataGyro1_CMS.Z + g_t_MpuDataGyro2_CMS.Z) / 2;
//	MPU6050_Data_Offset();
	g_v_LoopFilter(&g_t_Accel_CMS , &g_t_AccelAVG_CMS);
	g_v_ImuUpdate(&g_t_Gyro_CMS , &g_t_AccelAVG_CMS , &g_t_AttAngle_CMS);	
}

void g_v_Control(void)
{
	T_float_angle Angle;
	Angle.pit = g_t_AttAngle_CMS.pit - (g_t_Rc_D_CMS.PITCH - 1500) / 100;
	Angle.rol = g_t_AttAngle_CMS.rol - (g_t_Rc_D_CMS.ROLL - 1500)  / 100;
	
	g_fl_RolIntegral_CMS += Angle.rol;	//ROL积分
	if(g_fl_RolIntegral_CMS > 1800u)	//积分限幅
	{
		g_fl_RolIntegral_CMS = 1800u;
	}
	else if(g_fl_RolIntegral_CMS < -1800)
	{
		g_fl_RolIntegral_CMS = -1800;
	}
	//ROL PID output
	g_t_PidRol_CMS.PwmOutP = g_t_PidRol_CMS.P * Angle.rol;		
	g_t_PidRol_CMS.PwmOutD = g_t_PidRol_CMS.D * g_t_Gyro_CMS.Y;
	g_t_PidRol_CMS.PwmOutI = g_t_PidRol_CMS.I * g_fl_RolIntegral_CMS;
	
	g_fl_PitIntegral_CMS += Angle.pit;	//PIT积分
	if(g_fl_PitIntegral_CMS > 2000u)	//PIT积分限幅
	{
		g_fl_PitIntegral_CMS = 2000u;
	}
	else if(g_fl_PitIntegral_CMS < -2000)
	{
		g_fl_PitIntegral_CMS = -2000;
	}	
	//PIT PID output
	g_t_PidPit_CMS.PwmOutP = g_t_PidPit_CMS.P * Angle.pit;
	g_t_PidPit_CMS.PwmOutD = g_t_PidPit_CMS.D * g_t_Gyro_CMS.X;
	g_t_PidPit_CMS.PwmOutI = g_t_PidPit_CMS.I * g_fl_PitIntegral_CMS;
	//YAW在抖动范围之外才计算
	if(g_t_Rc_D_CMS.YAW < 1400 || g_t_Rc_D_CMS.YAW > 1600)	
	{
		g_t_Gyro_CMS.Z = g_t_Gyro_CMS.Z + (g_t_Rc_D_CMS.YAW - 1500) * 2;
	}
	g_fl_YawProportion_CMS += g_t_Gyro_CMS.Z * 0.0609756f * 0.002f;	//YAW比例
	
	if(g_fl_YawProportion_CMS > 20)	//YAW比例限幅
	{
		g_fl_YawProportion_CMS = 20;
	}
	if(g_fl_YawProportion_CMS < -20)
	{
		g_fl_YawProportion_CMS = -20;
	}
	//YAW PD out
	g_t_PidYaw_CMS.PwmOutP = g_t_PidYaw_CMS.P * g_fl_YawProportion_CMS;	
	g_t_PidYaw_CMS.PwmOutD = g_t_PidYaw_CMS.D * g_t_Gyro_CMS.Z;
	//油门小于1200时不动作
	if(g_t_Rc_D_CMS.THROTTLE < 1200)
	{
		g_fl_RolIntegral_CMS = 0;		
		g_fl_PitIntegral_CMS = 0;
		g_fl_YawProportion_CMS = 0;
	}

	g_t_PidRol_CMS.OUT = g_t_PidRol_CMS.PwmOutP + g_t_PidRol_CMS.PwmOutD + g_t_PidRol_CMS.PwmOutI;
	g_t_PidPit_CMS.OUT = g_t_PidPit_CMS.PwmOutP + g_t_PidPit_CMS.PwmOutD + g_t_PidPit_CMS.PwmOutI;
	g_t_PidYaw_CMS.OUT = g_t_PidYaw_CMS.PwmOutP + g_t_PidYaw_CMS.PwmOutD + g_t_PidYaw_CMS.PwmOutI;

	if(g_t_Rc_D_CMS.THROTTLE > 1200 && g_t_Rc_C_CMS.ARMED )
	{
		g_u16_Pwm1MC_CMS = g_t_Rc_D_CMS.THROTTLE - 1000u - g_t_PidRol_CMS.OUT + g_t_PidPit_CMS.OUT + g_t_PidYaw_CMS.OUT;
		g_u16_Pwm2MC_CMS = g_t_Rc_D_CMS.THROTTLE - 1000u + g_t_PidRol_CMS.OUT + g_t_PidPit_CMS.OUT - g_t_PidYaw_CMS.OUT;
		g_u16_Pwm3MC_CMS = g_t_Rc_D_CMS.THROTTLE - 1000u + g_t_PidRol_CMS.OUT - g_t_PidPit_CMS.OUT + g_t_PidYaw_CMS.OUT;
		g_u16_Pwm4MC_CMS = g_t_Rc_D_CMS.THROTTLE - 1000u - g_t_PidRol_CMS.OUT - g_t_PidPit_CMS.OUT - g_t_PidYaw_CMS.OUT;
	}
	else
	{
		g_u16_Pwm1MC_CMS = 0u;
		g_u16_Pwm2MC_CMS = 0u;
		g_u16_Pwm3MC_CMS = 0u;
		g_u16_Pwm4MC_CMS = 0u;
	}
	g_v_PwmUpdateDuty();	
}


void g_v_ControlDublePID(void)
{
	

		T_float_angle Angle;
	float temp=0;
//	/********************************************ROL PID***********************************************************/
//	if((g_t_Rc_D_CMS.ROLL > 1530) || (g_t_Rc_D_CMS.ROLL <1450))
//	{
//		temp = (float)(g_t_Rc_D_CMS.ROLL - 1500) / 20;
//	}
//	Angle.rol = temp  - g_t_AttAngle_CMS.rol  ;	//PIT??

//	g_fl_RolIntegralInternal_CMS +=g_t_PidRol_CMS.ExternalI * Angle.rol;
//	if(g_fl_RolIntegralInternal_CMS > 1000u)	//??????
//	{
//		g_fl_RolIntegralInternal_CMS = 1000u;
//	}
//	else if(g_fl_RolIntegralInternal_CMS < -1000)
//	{
//		g_fl_RolIntegralInternal_CMS = -1000;
//	}		
//	g_t_PidRol_CMS.PwmOutP=g_t_PidRol_CMS.InternalP *Angle.rol;
//	g_t_PidRol_CMS.PwmOutI=g_t_PidRol_CMS.InternalI *g_fl_RolIntegralInternal_CMS;
//	g_t_PidRol_CMS.PwmOutD=-g_t_PidRol_CMS.InternalD *g_t_Gyro_CMS.Y;
//	g_t_PidRol_CMS.InternalOut=g_t_PidRol_CMS.PwmOutP+g_t_PidRol_CMS.PwmOutD+g_t_PidRol_CMS.PwmOutI;

//	/********************************************ROL PID END*******************************************************/
//	
//	/********************************************PIT PID***********************************************************/
//	temp=0;
//	if((g_t_Rc_D_CMS.PITCH > 1530) || (g_t_Rc_D_CMS.PITCH <1450))
//	{
//		temp = (float)(g_t_Rc_D_CMS.PITCH - 1500) / 20;
//	}
//	Angle.pit = temp  - g_t_AttAngle_CMS.pit  ;	//PIT??

//	g_fl_PitIntegralInternal_CMS +=g_t_PidPit_CMS.ExternalI * Angle.pit;
//	if(g_fl_PitIntegralInternal_CMS > 2000u)	//??????
//	{
//		g_fl_PitIntegralInternal_CMS = 2000u;
//	}
//	else if(g_fl_PitIntegralInternal_CMS < -2000)
//	{
//		g_fl_PitIntegralInternal_CMS = -2000;
//	}		
//	g_t_PidPit_CMS.PwmOutP=g_t_PidPit_CMS.InternalP *Angle.pit;
//	g_t_PidPit_CMS.PwmOutI=g_t_PidPit_CMS.InternalI *g_fl_PitIntegralInternal_CMS;
//	g_t_PidPit_CMS.PwmOutD=g_t_PidPit_CMS.InternalD *g_t_Gyro_CMS.X;
//	g_t_PidPit_CMS.InternalOut=g_t_PidPit_CMS.PwmOutP+g_t_PidPit_CMS.PwmOutD+g_t_PidPit_CMS.PwmOutI;

//	/********************************************PIT PID END*******************************************************/
		/********************************************ROL PID***********************************************************/
	if((g_t_Rc_D_CMS.ROLL > 1530) || (g_t_Rc_D_CMS.ROLL <1450))
	{
		temp = (float)(g_t_Rc_D_CMS.ROLL - 1500) / 20;
	}
	Angle.rol =	temp - g_t_AttAngle_CMS.rol;	//ROL期望计算 / 12
//	Angle.rol = 0 - (g_t_Rc_D_CMS.ROLL - 1500) * 1.0  / 12;	//ROL期望计算
	
	g_fl_RolIntegralExternal_CMS += Angle.rol;	//ROL外环积分
	if(g_fl_RolIntegralExternal_CMS > 50u)	//外环积分限幅
	{
		g_fl_RolIntegralExternal_CMS = 50u;
	}
	else if(g_fl_RolIntegralExternal_CMS < -50)
	{
		g_fl_RolIntegralExternal_CMS = -50;
	}
	//ROL PID output
	g_t_PidRol_CMS.PwmOutP = g_t_PidRol_CMS.ExternalP * Angle.rol;		
	g_t_PidRol_CMS.PwmOutI = g_t_PidRol_CMS.ExternalI * g_fl_RolIntegralExternal_CMS;
	

	
	g_t_PidRol_CMS.ExternalOut = g_t_PidRol_CMS.PwmOutP + g_t_PidRol_CMS.PwmOutI;	//外环PI Output
	
	g_fl_RolIntegralInternal_CMS += (- g_t_Gyro_CMS.Y + g_t_PidRol_CMS.ExternalOut);	//ROL内环积分
	
	if(g_fl_RolIntegralInternal_CMS > 50u)	//内环积分限幅
	{
		g_fl_RolIntegralInternal_CMS = 50u;
	}
	else if(g_fl_RolIntegralInternal_CMS < -50)
	{
		g_fl_RolIntegralInternal_CMS = -50;
	}	
	//ROL内环PID Output
	g_t_PidRol_CMS.InternalOut =  g_t_PidRol_CMS.InternalP * (- g_t_Gyro_CMS.Y + g_t_PidRol_CMS.ExternalOut)	\
							    + g_t_PidRol_CMS.InternalI * g_fl_RolIntegralInternal_CMS					\
							    + g_t_PidRol_CMS.InternalD * (g_t_Gyro_CMS.Y - g_t_Gyro_CMS.LastY);
	g_t_Gyro_CMS.LastY = g_t_Gyro_CMS.Y;
	/********************************************ROL PID END*******************************************************/
	
	/********************************************PIT PID***********************************************************/
	temp=0;
	if((g_t_Rc_D_CMS.PITCH > 1530) || (g_t_Rc_D_CMS.PITCH <1450))
	{
		temp = (float)(g_t_Rc_D_CMS.PITCH - 1500) / 20;
	}
	Angle.pit = temp  - g_t_AttAngle_CMS.pit  ;	//PIT期望
//	Angle.pit = 0 - (g_t_Rc_D_CMS.PITCH - 1500) * 1.0 / 12;	//PIT期望
	g_fl_PitIntegralExternal_CMS += Angle.pit;	//外环PIT积分
	if(g_fl_PitIntegralExternal_CMS > 50)	//PIT积分限幅
	{
		g_fl_PitIntegralExternal_CMS = 50;
	}
	else if(g_fl_PitIntegralExternal_CMS < -50)
	{
		g_fl_PitIntegralExternal_CMS = -50;
	}	
	//PIT PID output
	g_t_PidPit_CMS.PwmOutP = g_t_PidPit_CMS.ExternalP * Angle.pit;
	g_t_PidPit_CMS.PwmOutI = g_t_PidPit_CMS.ExternalI * g_fl_PitIntegralExternal_CMS;


	
	g_t_PidPit_CMS.ExternalOut = g_t_PidPit_CMS.PwmOutP + g_t_PidPit_CMS.PwmOutI;	//外环PI Output
	
	g_fl_PitIntegralInternal_CMS += (- g_t_Gyro_CMS.X + g_t_PidPit_CMS.ExternalOut);	//PIT内环积分
	
	if(g_fl_PitIntegralInternal_CMS > 50u)	//内环积分限幅
	{
		g_fl_PitIntegralInternal_CMS = 50u;
	}
	else if(g_fl_PitIntegralInternal_CMS < -50)
	{
		g_fl_PitIntegralInternal_CMS = -50;
	}	
	//PIT内环PID Output
	g_t_PidPit_CMS.InternalOut = g_t_PidPit_CMS.InternalP * (- g_t_Gyro_CMS.X + g_t_PidPit_CMS.ExternalOut)	\
							   + g_t_PidPit_CMS.InternalI * g_fl_PitIntegralInternal_CMS					\
							   + g_t_PidPit_CMS.InternalD * (g_t_Gyro_CMS.X - g_t_Gyro_CMS.LastX);
	g_t_Gyro_CMS.LastX = g_t_Gyro_CMS.X;
	/********************************************PIT PID END*******************************************************/
	/********************************************YAW PID***********************************************************/
	//YAW在抖动范围之外才计算
	if(g_t_Rc_D_CMS.YAW < 1400 || g_t_Rc_D_CMS.YAW > 1600)	
	{
		g_t_Gyro_CMS.Z = g_t_Gyro_CMS.Z + (g_t_Rc_D_CMS.YAW - 1500) * 2;
//		g_t_Gyro_CMS.Z = 0 + (g_t_Rc_D_CMS.YAW - 1500) * 2;
	}
	g_fl_YawProportion_CMS += g_t_Gyro_CMS.Z * 0.0609756f * 0.002f;	//YAW比例
	
	if(g_fl_YawProportion_CMS > 20)	//YAW比例限幅
	{
		g_fl_YawProportion_CMS = 20;
	}
	if(g_fl_YawProportion_CMS < -20)
	{
		g_fl_YawProportion_CMS = -20;
	}
	//YAW PD out
	g_t_PidYaw_CMS.PwmOutP = g_t_PidYaw_CMS.P * g_fl_YawProportion_CMS;	
	g_t_PidYaw_CMS.PwmOutD = g_t_PidYaw_CMS.D * g_t_Gyro_CMS.Z;	
	g_t_PidYaw_CMS.OUT = (g_t_PidYaw_CMS.PwmOutP + g_t_PidYaw_CMS.PwmOutD);
	/********************************************YAW PID END*******************************************************/
	
	//油门小于1200时不动作
	if(g_t_Rc_D_CMS.THROTTLE < 1200)
	{
		g_fl_RolIntegral_CMS = 0;		
		g_fl_PitIntegral_CMS = 0;
		g_fl_YawProportion_CMS = 0;
	}
//	if(g_t_Rc_C_CMS.ARMED )
//	if(g_t_Rc_D_CMS.THROTTLE > 1200  )
	if(g_t_Rc_D_CMS.THROTTLE > 1200 && g_t_Rc_C_CMS.ARMED )
	{
		g_u16_Pwm1MC_CMS = g_t_Rc_D_CMS.THROTTLE - 1000u - g_t_PidRol_CMS.InternalOut + g_t_PidPit_CMS.InternalOut + g_t_PidYaw_CMS.OUT;
		g_u16_Pwm2MC_CMS = g_t_Rc_D_CMS.THROTTLE - 1000u + g_t_PidRol_CMS.InternalOut + g_t_PidPit_CMS.InternalOut - g_t_PidYaw_CMS.OUT;
		g_u16_Pwm3MC_CMS = g_t_Rc_D_CMS.THROTTLE - 1000u + g_t_PidRol_CMS.InternalOut - g_t_PidPit_CMS.InternalOut + g_t_PidYaw_CMS.OUT;
		g_u16_Pwm4MC_CMS = g_t_Rc_D_CMS.THROTTLE - 1000u - g_t_PidRol_CMS.InternalOut - g_t_PidPit_CMS.InternalOut - g_t_PidYaw_CMS.OUT;
//		temp_rolpwmP = (int16_t)g_t_PidRol_CMS.ExternalOut;
		temp_rolpwmI = (int16_t)g_t_PidPit_CMS.ExternalOut;

//	temp_pitpwmI = (int16_t)g_t_PidPit_CMS.PwmOutI;
	}
	else
	{
		g_u16_Pwm1MC_CMS = 0u;
		g_u16_Pwm2MC_CMS = 0u;
		g_u16_Pwm3MC_CMS = 0u;
		g_u16_Pwm4MC_CMS = 0u;
	}
	g_v_PwmUpdateDuty();	
}








void g_v_ControlDublePID2(void)
{
	T_float_angle Angle;
	float temp=0,temp_throttle=0 ,temp1=0;
	/********************************************ROL PID***********************************************************/
	if((g_t_Rc_D_CMS.ROLL > 1530))
	{
		temp = (float)(g_t_Rc_D_CMS.ROLL - 1530) / RC_CTRL;

	}
	if(g_t_Rc_D_CMS.ROLL < 1450)
	{
		temp = (float)(g_t_Rc_D_CMS.ROLL - 1450) / RC_CTRL;
	}
	Angle.rol =	temp - g_t_AttAngle_CMS.rol;	//ROL期望计算 / 12
//	Angle.rol = 0 - (g_t_Rc_D_CMS.ROLL - 1500) * 1.0  / 12;	//ROL期望计算
	
	g_fl_RolIntegralExternal_CMS += Angle.rol;	//ROL外环积分
	if(g_fl_RolIntegralExternal_CMS > IntegralExternal)	//外环积分限幅
	{
		g_fl_RolIntegralExternal_CMS = IntegralExternal;
	}
	else if(g_fl_RolIntegralExternal_CMS < -IntegralExternal)
	{
		g_fl_RolIntegralExternal_CMS = -IntegralExternal;
	}
	//ROL PID output
	g_t_PidRol_CMS.PwmOutP = g_t_PidRol_CMS.ExternalP * Angle.rol;		
	g_t_PidRol_CMS.PwmOutI = g_t_PidRol_CMS.ExternalI * g_fl_RolIntegralExternal_CMS;
	

	
	g_t_PidRol_CMS.ExternalOut = g_t_PidRol_CMS.PwmOutP + g_t_PidRol_CMS.PwmOutI;	//外环PI Output
	
	g_fl_RolIntegralInternal_CMS += ( g_t_Gyro_CMS.Y + g_t_PidRol_CMS.ExternalOut);	//ROL内环积分
	
	if(g_fl_RolIntegralInternal_CMS > IntegralInternal)	//内环积分限幅
	{
		g_fl_RolIntegralInternal_CMS = IntegralInternal;
	}
	else if(g_fl_RolIntegralInternal_CMS < -IntegralInternal)
	{
		g_fl_RolIntegralInternal_CMS = -IntegralInternal;
	}	
	//ROL内环PID Output
	g_t_PidRol_CMS.InternalOut =  g_t_PidRol_CMS.InternalP * (g_t_Gyro_CMS.Y + g_t_PidRol_CMS.ExternalOut)	\
							    + g_t_PidRol_CMS.InternalI * g_fl_RolIntegralInternal_CMS					\
							    + g_t_PidRol_CMS.InternalD * (g_t_Gyro_CMS.Y + g_t_PidRol_CMS.ExternalOut - g_t_Gyro_CMS.LastY);
	g_t_Gyro_CMS.LastY = g_t_Gyro_CMS.Y + g_t_PidRol_CMS.ExternalOut;
	/********************************************ROL PID END*******************************************************/
	
	/********************************************PIT PID***********************************************************/
	temp=0;
	if((g_t_Rc_D_CMS.PITCH > 1530))
	{
		temp = (float)(g_t_Rc_D_CMS.PITCH - 1530) / RC_CTRL;

	}
	if((g_t_Rc_D_CMS.PITCH <1450))
	{
		temp = (float)(g_t_Rc_D_CMS.PITCH - 1450) / RC_CTRL;

	}
	Angle.pit = temp  - g_t_AttAngle_CMS.pit  ;	//PIT期望
//	Angle.pit = 0 - (g_t_Rc_D_CMS.PITCH - 1500) * 1.0 / 12;	//PIT期望
	g_fl_PitIntegralExternal_CMS += Angle.pit;	//外环PIT积分
	if(g_fl_PitIntegralExternal_CMS > IntegralExternal)	//PIT积分限幅
	{
		g_fl_PitIntegralExternal_CMS = IntegralExternal;
	}
	else if(g_fl_PitIntegralExternal_CMS < -IntegralExternal)
	{
		g_fl_PitIntegralExternal_CMS = -IntegralExternal;
	}	
	//PIT PID output
	g_t_PidPit_CMS.PwmOutP = g_t_PidPit_CMS.ExternalP * Angle.pit;
	g_t_PidPit_CMS.PwmOutI = g_t_PidPit_CMS.ExternalI * g_fl_PitIntegralExternal_CMS;


	
	g_t_PidPit_CMS.ExternalOut = g_t_PidPit_CMS.PwmOutP + g_t_PidPit_CMS.PwmOutI;	//外环PI Output
	
	g_fl_PitIntegralInternal_CMS +=(- g_t_Gyro_CMS.X + g_t_PidPit_CMS.ExternalOut);	//PIT内环积分40.5
	
	if(g_fl_PitIntegralInternal_CMS > IntegralInternal)	//内环积分限幅
	{
		g_fl_PitIntegralInternal_CMS = IntegralInternal;
	}
	else if(g_fl_PitIntegralInternal_CMS < -IntegralInternal)
	{
		g_fl_PitIntegralInternal_CMS = -IntegralInternal;
	}	
	//PIT内环PID Output
	g_t_PidPit_CMS.InternalOut = g_t_PidPit_CMS.InternalP * (- g_t_Gyro_CMS.X + g_t_PidPit_CMS.ExternalOut)	\
							   + g_t_PidPit_CMS.InternalI *  g_fl_PitIntegralInternal_CMS					\
							   + g_t_PidPit_CMS.InternalD * (- g_t_Gyro_CMS.X + g_t_PidPit_CMS.ExternalOut - g_t_Gyro_CMS.LastX);
	g_t_Gyro_CMS.LastX = - g_t_Gyro_CMS.X + g_t_PidPit_CMS.ExternalOut;
	/********************************************PIT PID END*******************************************************/
	
	/********************************************YAW PID***********************************************************/
	temp=0;
	//YAW在抖动范围之外才计算
	if(g_t_Rc_D_CMS.YAW < 1400 || g_t_Rc_D_CMS.YAW > 1600)	
	{
		g_t_Gyro_CMS.Z = g_t_Gyro_CMS.Z + (g_t_Rc_D_CMS.YAW - 1500) * 2;
//		temp = (float)(g_t_Rc_D_CMS.YAW - 1500) * 2;
	}
	g_fl_YawProportion_CMS += g_t_Gyro_CMS.Z * 0.0609756f * 0.002f;	//YAW比例
	
	if(g_fl_YawProportion_CMS > 20)	//YAW比例限幅
	{
		g_fl_YawProportion_CMS = 20;
	}
	if(g_fl_YawProportion_CMS < -20)
	{
		g_fl_YawProportion_CMS = -20;
	}
	//YAW PD out
	g_t_PidYaw_CMS.PwmOutP = g_t_PidYaw_CMS.P * g_fl_YawProportion_CMS;	
	g_t_PidYaw_CMS.PwmOutD = g_t_PidYaw_CMS.D * g_t_Gyro_CMS.Z;	
	g_t_PidYaw_CMS.OUT = (g_t_PidYaw_CMS.PwmOutP + g_t_PidYaw_CMS.PwmOutD);
	/********************************************YAW PID END*******************************************************/
	
	//油门小于1200时不动作
	if(g_t_Rc_D_CMS.THROTTLE < 1200)
	{
		g_fl_PitIntegralInternal_CMS = 0;
		g_fl_PitIntegralExternal_CMS = 0;	
		g_fl_RolIntegralInternal_CMS = 0;
		g_fl_RolIntegralExternal_CMS = 0;
		g_fl_YawProportion_CMS = 0;
	}
//	if(g_t_Rc_C_CMS.ARMED )
//	if(g_t_Rc_D_CMS.THROTTLE > 1200  )

	temp_throttle = g_t_Rc_D_CMS.THROTTLE;
//	temp1 = sqrt(g_t_AttAngle_CMS.rol * g_t_AttAngle_CMS.rol + g_t_AttAngle_CMS.pit *g_t_AttAngle_CMS.pit);///cos(g_t_AttAngle_CMS.rol/57.3)/cos(g_t_AttAngle_CMS.pit/57.3);   
//	temp_throttle += temp_throttle * (1.0f - cos(temp1 * 0.0174533f));	
	if((temp_throttle > 1200) && g_t_Rc_C_CMS.ARMED )
	{
		g_u16_Pwm1MC_CMS = g_t_Rc_D_CMS.THROTTLE - 1200u - g_t_PidRol_CMS.InternalOut - g_t_PidPit_CMS.InternalOut + g_t_PidYaw_CMS.OUT;
		g_u16_Pwm2MC_CMS = g_t_Rc_D_CMS.THROTTLE - 1200u + g_t_PidRol_CMS.InternalOut - g_t_PidPit_CMS.InternalOut - g_t_PidYaw_CMS.OUT;
		g_u16_Pwm3MC_CMS = g_t_Rc_D_CMS.THROTTLE - 1200u + g_t_PidRol_CMS.InternalOut + g_t_PidPit_CMS.InternalOut + g_t_PidYaw_CMS.OUT;
		g_u16_Pwm4MC_CMS = g_t_Rc_D_CMS.THROTTLE - 1200u - g_t_PidRol_CMS.InternalOut + g_t_PidPit_CMS.InternalOut - g_t_PidYaw_CMS.OUT;
//		temp_rolpwmP = (int16_t)g_t_PidRol_CMS.ExternalOut;
//		temp_rolpwmI = (int16_t)g_t_PidPit_CMS.ExternalOut;

//	temp_pitpwmI = (int16_t)g_t_PidPit_CMS.PwmOutI;
	}
	else
	{
		g_u16_Pwm1MC_CMS = 0u;
		g_u16_Pwm2MC_CMS = 0u;
		g_u16_Pwm3MC_CMS = 0u;
		g_u16_Pwm4MC_CMS = 0u;
	}
	g_v_PwmUpdateDuty();	
}












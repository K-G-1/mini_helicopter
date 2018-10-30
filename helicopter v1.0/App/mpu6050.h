#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"
#include "appconfig.h"
#include "data.h"
/* MPU6050 Addresses defines */
#define MPU6050_ADDRESS 0xD0   	//IIC写入时的地址字节数据，+1为读取



//****************************************
// 定义MPU6050内部地址
//****************************************

#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	MPU_CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)

extern volatile uint8_t g_u8a14_MpuRegBuf_SMP[14];

//以下定义传感器输出数据存放在mpu_reg_buf中的位置
#define ACCEL_XOUT_H_buf_add	0   
#define ACCEL_XOUT_L_buf_add	1  
#define ACCEL_YOUT_H_buf_add	2 
#define ACCEL_YOUT_L_buf_add	3 
#define ACCEL_ZOUT_H_buf_add	4 
#define ACCEL_ZOUT_L_buf_add	5 
#define TEMP_OUT_H_buf_add   	6 
#define TEMP_OUT_L_buf_add  	7 
#define GYRO_XOUT_H_buf_add	    8 
#define GYRO_XOUT_L_buf_add		9 
#define GYRO_YOUT_H_buf_add		10
#define GYRO_YOUT_L_buf_add		11
#define GYRO_ZOUT_H_buf_add		12
#define GYRO_ZOUT_L_buf_add		13
#define ACC_CAL_EN

#define OFFSET_AV_NUM 50
#define ACC_ADJ_EN //????????
typedef struct 
{
	u8 acc_CALIBRATE;
	u8 gyr_CALIBRATE;
	u8 vec3d_CALIBRATE;
	
	_xyz_s16_st Acc_I16;
	_xyz_s16_st Gyro_I16;

	_xyz_f_st Acc;
	_xyz_f_st Acc_mmss;
	_xyz_f_st Gyro;
	_xyz_f_st Gyro_deg;
	
	float Gyro_Temprea_Adjust;
	float ACC_Temprea_Adjust;

	s16 Tempreature;
	float Ftempreature;
	
}_sensor_st;

enum
{
 A_X = 0,
 A_Y ,
 A_Z ,
 G_Y ,
 G_X ,
 G_Z ,
 TEM ,
 ITEMS ,
};


extern _sensor_st sensor;
uint8_t check_mpu6050(void);
void mpu6050_write_byte(uint8_t reg_address,uint8_t data);
void mpu6050_write_reg(uint8_t reg_address , uint8_t n);
uint8_t mpu6050_read_byte(uint8_t reg_address);
void mpu6050_read_reg(uint8_t reg_address ,uint8_t first_add, uint8_t n);
void mpu6050_init(void);
void MPU6050_Data_Offset(void);
//void MPU6050_Dataanl(T_int16_xyz *data_tempacc,T_int16_xyz *data_tempgyr);
#endif


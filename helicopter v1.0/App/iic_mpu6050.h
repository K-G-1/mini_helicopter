#ifndef __IIC_MPU6050_H
#define __IIC_MPU6050_H


 
#include "stm32f10x.h"


#define I2C_Speed              400000
#define I2C1_OWN_ADDRESS7      0x0d

/* MPU6050 Addresses defines */
#define MPU6050_Block0_ADDRESS 0xD0   	//IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ
//#define MPU6050_Block1_ADDRESS 0xD2 


//****************************************
// ����MPU6050�ڲ���ַ
//****************************************

#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
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
#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)



void I2C_MPU_Init(void);
void I2C_MPU_ByteWrite(u8 pBuffer, u8 WriteAddr);
u8 I2C_MPU_BufferRead( u8 ReadAddr);
int mpu6050_get(u8 REG_Address);



#define IIC_START()  GPIO_SetBits(GPIOB, GPIO_Pin_6);\
					 GPIO_SetBits(GPIOB, GPIO_Pin_7);\
					 GPIO_ResetBits(GPIOB, GPIO_Pin_7)

#define IIC_STOP()   GPIO_SetBits(GPIOB, GPIO_Pin_6);\
					 GPIO_ResetBits(GPIOB, GPIO_Pin_7);\
					 GPIO_SetBits(GPIOB, GPIO_Pin_7)
					 


#endif




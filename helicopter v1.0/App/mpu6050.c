
#include "mpu6050.h"
#include "i2c_gpio.h"
#include "usart.h"
#include "led.h"
#include "SysTick.h"
#include "delay.h"
#include "param.h"
#include "DataTransferV26.h"
#include "control.h"
//连续读取字节时存放数据的地址
volatile uint8_t g_u8a14_MpuRegBuf_SMP[14];
		
//初始化SMPLRT_DIV、CONFIG、GYRO_CONFIG、ACCEL_CONFIG寄存器
uint8_t g_u8a4_MpuRegData_SMP[]={0x07,0x06,0x18,0x01};

	
_sensor_st sensor;

//检测mpu6050是否插好
uint8_t check_mpu6050(void)
{
	uint8_t res;
	//检测到应答信号,配置了gpio
	if(i2c_CheckDevice(MPU6050_ADDRESS)==0)	
	{
		USART_printf(USART1,(u8 *)"检测mpu6050成功\r\n");
		LED1(OFF);
		res = 1;
	}
	else //没检测到应答信号
	{
		USART_printf(USART1,(u8 *)"mpu6050没插好\r\n");
		LED1(ON);
		res = 0;	
	}
	return res;
}

//向mpu6050特定地址写一个字节的数据
void mpu6050_write_byte(uint8_t reg_address,uint8_t data)
{
	uint8_t ucAck;
	i2c_Start();
	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	i2c_SendByte(MPU6050_ADDRESS | I2C_WR);
	/* 检测ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
	/* 发送寄存器地址 */
	i2c_SendByte(reg_address);
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
	i2c_SendByte(data);
	/* 检测ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
err_quit:
	i2c_Stop();
		
}

/**********************************************************

 连续写寄存器子函数

reg_address	：需要连续写的寄存器的首地址
n			：需要连续写寄存器的数量
mpu_reg_data：存放需要写入寄存器的数据
**********************************************************/
void mpu6050_write_reg(uint8_t reg_address , uint8_t n)
{
	uint8_t i;
	uint8_t ucAck;
	i2c_Start();
	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	i2c_SendByte(MPU6050_ADDRESS | I2C_WR);
	/* 检测ACK */
    ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
	/* 发送寄存器地址 */
	i2c_SendByte(reg_address);
	/* 检测ACK */
    ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
	// 寄存器连续写操作
	for(i=0; i<n; i++)
	{
		i2c_SendByte(g_u8a4_MpuRegData_SMP[i]);
		/* 检测ACK */
		ucAck = i2c_WaitAck();
		if (ucAck == 1)
		{
			goto err_quit;
		}
	}
err_quit:
	i2c_Stop();
}

//读寄存器数据
uint8_t mpu6050_read_byte(uint8_t reg_address)
{
	uint8_t ucAck;
	uint8_t	mpu_reg_buf;
	i2c_Start();
	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	i2c_SendByte(MPU6050_ADDRESS | I2C_WR);
	/* 检测ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
	/* 发送寄存器 */
	i2c_SendByte(reg_address);
	/* 检测ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
	i2c_Start();
	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	i2c_SendByte(MPU6050_ADDRESS | I2C_RD);
	/* 检测ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
	mpu_reg_buf = i2c_ReadByte();
	i2c_NAck();
err_quit:
	i2c_Stop();
	return mpu_reg_buf;
}

/**********************************************************

 连续读寄存器子函数
reg_address	：需要连续读的寄存器的首地址
first_add   ：需要存放在数据缓冲mpu_reg_buf中的地址
n			：需要连续读寄存器的数量

**********************************************************/

void mpu6050_read_reg(uint8_t reg_address ,uint8_t first_add, uint8_t n)
{
	uint8_t ucAck;
	uint8_t i;
	i2c_Start();
	i2c_SendByte(MPU6050_ADDRESS | I2C_WR);/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	ucAck = i2c_WaitAck();/* 检测ACK */
	if (ucAck == 1)
	{
		goto err_quit;
	}	
	i2c_SendByte(reg_address);/* 发送寄存器地址 */
	ucAck = i2c_WaitAck();/* 检测ACK */
	if (ucAck == 1)
	{
		goto err_quit;
	}
	i2c_Start();
	i2c_SendByte(MPU6050_ADDRESS | I2C_RD);/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */	
	ucAck = i2c_WaitAck();/* 检测ACK */
	if (ucAck == 1)
	{
		goto err_quit;
	}
	for (i = first_add; i < n+first_add; i++)
	{
		g_u8a14_MpuRegBuf_SMP[i] = i2c_ReadByte();
		if (i == n+first_add-1)
		{
			i2c_NAck();
		}
		else
		{                        
			i2c_Ack();
		}
	}
err_quit:
	i2c_Stop();
}

void mpu6050_init(void)
{
	mpu6050_write_byte(PWR_MGMT_1, 0x00);
	delayms(3000);
  mpu6050_write_byte(PWR_MGMT_1, 0x01);
/**********以下是单个字节写寄存器*************/
//	mpu6050_write_byte(SMPLRT_DIV, 0x07);	
	
	mpu6050_write_byte(MPU_CONFIG, 0x03);
	delayms(500);
	mpu6050_write_byte(GYRO_CONFIG, 0x18);	//+-2000度
	delayms(500);
	mpu6050_write_byte(ACCEL_CONFIG, 0x09); //+-4G、0.5Hz
	delayms(500);
}
/********************************************/
	

/***********连续写以上寄存器*****************/
//	mpu6050_write_reg(ACCEL_CONFIG, 0x01);


s32 sum_temp[7]={0,0,0,0,0,0,0};
u16 acc_sum_cnt = 0,gyro_sum_cnt = 0;
void MPU6050_Data_Offset(void)
{
	if(sensor.gyr_CALIBRATE)
	{
		gyro_sum_cnt++;
		sum_temp[G_X] += g_t_Gyro_CMS.X;
		sum_temp[G_Y] += g_t_Gyro_CMS.Y;
		sum_temp[G_Z] += g_t_Gyro_CMS.Z;
		sum_temp[TEM] += sensor.Tempreature;

    if( gyro_sum_cnt >= OFFSET_AV_NUM )
		{
			ANO_Param.gyr_offset.x = (float)sum_temp[G_X]/OFFSET_AV_NUM;
			ANO_Param.gyr_offset.y = (float)sum_temp[G_Y]/OFFSET_AV_NUM;
			ANO_Param.gyr_offset.z = (float)sum_temp[G_Z]/OFFSET_AV_NUM;
			ANO_Param.gyr_temprea_offset = sum_temp[TEM]/OFFSET_AV_NUM;
			gyro_sum_cnt =0;
			if(sensor.gyr_CALIBRATE == 1)
			{
				ANO_Param_Save();
				
			}
			sensor.gyr_CALIBRATE = 0;
			f.msg_id = 2;
			f.msg_data = 1;
//			LED_warn = 2; //????
			sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
		}
}
	#ifdef ACC_CAL_EN

	if(sensor.acc_CALIBRATE == 1)
	{
    acc_sum_cnt++;
		sum_temp[A_X] += g_t_Accel_CMS.X;
		sum_temp[A_Y] += g_t_Accel_CMS.Y;
		sum_temp[A_Z] += g_t_Accel_CMS.Z;// - 65535/16;   // +-8G
		sum_temp[TEM] += sensor.Tempreature;

    if( acc_sum_cnt >= OFFSET_AV_NUM )
		{
			ANO_Param.acc_offset.x = sum_temp[A_X]/OFFSET_AV_NUM;
			ANO_Param.acc_offset.y = sum_temp[A_Y]/OFFSET_AV_NUM;
			ANO_Param.acc_offset.z = sum_temp[A_Z]/OFFSET_AV_NUM;
			
//			mpu_type(ANO_Param.acc_offset.x,ANO_Param.acc_offset.y,ANO_Param.acc_offset.z);//6050????(????)
			
			ANO_Param.acc_temprea_offset = sum_temp[TEM]/OFFSET_AV_NUM;
			acc_sum_cnt =0;
			sensor.acc_CALIBRATE = 0;
			f.msg_id = 1;
			f.msg_data = 1;
			ANO_Param_Save();
//			LED_warn = 3;//????
			sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;
		}	
	}
	#endif
}






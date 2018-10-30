
#include "mpu6050.h"
#include "i2c_gpio.h"
#include "usart.h"
#include "led.h"
#include "SysTick.h"
#include "delay.h"
#include "param.h"
#include "DataTransferV26.h"
#include "control.h"
//������ȡ�ֽ�ʱ������ݵĵ�ַ
volatile uint8_t g_u8a14_MpuRegBuf_SMP[14];
		
//��ʼ��SMPLRT_DIV��CONFIG��GYRO_CONFIG��ACCEL_CONFIG�Ĵ���
uint8_t g_u8a4_MpuRegData_SMP[]={0x07,0x06,0x18,0x01};

	
_sensor_st sensor;

//���mpu6050�Ƿ���
uint8_t check_mpu6050(void)
{
	uint8_t res;
	//��⵽Ӧ���ź�,������gpio
	if(i2c_CheckDevice(MPU6050_ADDRESS)==0)	
	{
		USART_printf(USART1,(u8 *)"���mpu6050�ɹ�\r\n");
		LED1(OFF);
		res = 1;
	}
	else //û��⵽Ӧ���ź�
	{
		USART_printf(USART1,(u8 *)"mpu6050û���\r\n");
		LED1(ON);
		res = 0;	
	}
	return res;
}

//��mpu6050�ض���ַдһ���ֽڵ�����
void mpu6050_write_byte(uint8_t reg_address,uint8_t data)
{
	uint8_t ucAck;
	i2c_Start();
	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	i2c_SendByte(MPU6050_ADDRESS | I2C_WR);
	/* ���ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
	/* ���ͼĴ�����ַ */
	i2c_SendByte(reg_address);
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
	i2c_SendByte(data);
	/* ���ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
err_quit:
	i2c_Stop();
		
}

/**********************************************************

 ����д�Ĵ����Ӻ���

reg_address	����Ҫ����д�ļĴ������׵�ַ
n			����Ҫ����д�Ĵ���������
mpu_reg_data�������Ҫд��Ĵ���������
**********************************************************/
void mpu6050_write_reg(uint8_t reg_address , uint8_t n)
{
	uint8_t i;
	uint8_t ucAck;
	i2c_Start();
	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	i2c_SendByte(MPU6050_ADDRESS | I2C_WR);
	/* ���ACK */
    ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
	/* ���ͼĴ�����ַ */
	i2c_SendByte(reg_address);
	/* ���ACK */
    ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
	// �Ĵ�������д����
	for(i=0; i<n; i++)
	{
		i2c_SendByte(g_u8a4_MpuRegData_SMP[i]);
		/* ���ACK */
		ucAck = i2c_WaitAck();
		if (ucAck == 1)
		{
			goto err_quit;
		}
	}
err_quit:
	i2c_Stop();
}

//���Ĵ�������
uint8_t mpu6050_read_byte(uint8_t reg_address)
{
	uint8_t ucAck;
	uint8_t	mpu_reg_buf;
	i2c_Start();
	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	i2c_SendByte(MPU6050_ADDRESS | I2C_WR);
	/* ���ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
	/* ���ͼĴ��� */
	i2c_SendByte(reg_address);
	/* ���ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		goto err_quit;
	}
	i2c_Start();
	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	i2c_SendByte(MPU6050_ADDRESS | I2C_RD);
	/* ���ACK */
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

 �������Ĵ����Ӻ���
reg_address	����Ҫ�������ļĴ������׵�ַ
first_add   ����Ҫ��������ݻ���mpu_reg_buf�еĵ�ַ
n			����Ҫ�������Ĵ���������

**********************************************************/

void mpu6050_read_reg(uint8_t reg_address ,uint8_t first_add, uint8_t n)
{
	uint8_t ucAck;
	uint8_t i;
	i2c_Start();
	i2c_SendByte(MPU6050_ADDRESS | I2C_WR);/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	ucAck = i2c_WaitAck();/* ���ACK */
	if (ucAck == 1)
	{
		goto err_quit;
	}	
	i2c_SendByte(reg_address);/* ���ͼĴ�����ַ */
	ucAck = i2c_WaitAck();/* ���ACK */
	if (ucAck == 1)
	{
		goto err_quit;
	}
	i2c_Start();
	i2c_SendByte(MPU6050_ADDRESS | I2C_RD);/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */	
	ucAck = i2c_WaitAck();/* ���ACK */
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
/**********�����ǵ����ֽ�д�Ĵ���*************/
//	mpu6050_write_byte(SMPLRT_DIV, 0x07);	
	
	mpu6050_write_byte(MPU_CONFIG, 0x03);
	delayms(500);
	mpu6050_write_byte(GYRO_CONFIG, 0x18);	//+-2000��
	delayms(500);
	mpu6050_write_byte(ACCEL_CONFIG, 0x09); //+-4G��0.5Hz
	delayms(500);
}
/********************************************/
	

/***********����д���ϼĴ���*****************/
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






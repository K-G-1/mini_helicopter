//#include "include.h"
#include "mpu6050.h"
#include "mpu9250.h"
#include "delay.h"
#include "IIC.h"
#include "Algorithm_filter.h"
#include "param.h"

u8		 mpu6050_buffer[14];	



u8 mpu6050_init(void)
{
    u8 res;
    IIC_ADD_write(MPU_6050_ADDR,MPU_PWR_MGMT1_REG,0x00); //Reset ing
    delay_ms(10);
    IIC_ADD_write(MPU_6050_ADDR,MPU_PWR_MGMT1_REG,0x03); //Reset ing
	delay_ms(10);
	IIC_ADD_write(MPU_6050_ADDR,MPU_SAMPLE_RATE_REG,0x00);
	delay_ms(10);					 
    IIC_ADD_write(MPU_6050_ADDR,MPU_CFG_REG,0x03);  
	delay_ms(10);
	IIC_ADD_write(MPU_6050_ADDR,MPU_ACCEL_CFG_REG,0x10);//加速度度最大量程 +-8G
	delay_ms(10);	
	IIC_ADD_write(MPU_6050_ADDR,MPU_GYRO_CFG_REG,0x10); //陀螺仪最大量程 +-2000度每秒
	delay_ms(10);

    
////    IIC_ADD_write(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
////    delay_ms(10);	
////    IIC_ADD_write(MAG_ADDRESS,0x0A,0x01);
////    delay_ms(10);	
    
    res=IIC_ADD_read(MPU_6050_ADDR,MPU_DEVICE_ID_REG);
    
    if(res==0x68)//器件ID正确
	{
		return 0;
 	}else 
	{
		return 1;
	}
}

//******读取MPU9250数据****************************************
void READ_6050()
{
    IIC_Read_MultiBytes(MPU_6050_ADDR,MPU_ACCEL_XOUTH_REG,14,mpu6050_buffer);
    sensor.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]);
	sensor.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]);
	sensor.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) ;
    sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
    
//    READ_MPU6050_ACCEL();
//    READ_MPU6050_GYRO();
}
void READ_MPU6050_ACCEL(void)
{ 
    
    mpu6050_buffer[0]=IIC_ADD_read(MPU_6050_ADDR,MPU_ACCEL_XOUTL_REG); 
    mpu6050_buffer[1]=IIC_ADD_read(MPU_6050_ADDR,MPU_ACCEL_XOUTH_REG);
    sensor.acc.origin.x=	((int16_t)mpu6050_buffer[1]<<8)|mpu6050_buffer[0];             //读取计算X轴数据

    mpu6050_buffer[2]=IIC_ADD_read(MPU_6050_ADDR,MPU_ACCEL_YOUTL_REG);
    mpu6050_buffer[3]=IIC_ADD_read(MPU_6050_ADDR,MPU_ACCEL_YOUTH_REG);
    sensor.acc.origin.y =	((int16_t)mpu6050_buffer[3]<<8)|mpu6050_buffer[2];             //读取计算Y轴数据
    
    mpu6050_buffer[4]=IIC_ADD_read(MPU_6050_ADDR,MPU_ACCEL_ZOUTL_REG);
    mpu6050_buffer[5]=IIC_ADD_read(MPU_6050_ADDR,MPU_ACCEL_ZOUTH_REG);
    sensor.acc.origin.z =	((int16_t)mpu6050_buffer[5]<<8)|mpu6050_buffer[4];             //读取计算Z轴数据
 
}

void READ_MPU6050_GYRO(void)
{ 
    
    mpu6050_buffer[8]=IIC_ADD_read(MPU_6050_ADDR,MPU_GYRO_XOUTL_REG); 
    mpu6050_buffer[9]=IIC_ADD_read(MPU_6050_ADDR,MPU_GYRO_XOUTH_REG);
    sensor.gyro.origin.x =	((int16_t)mpu6050_buffer[9]<<8)|mpu6050_buffer[8];

    mpu6050_buffer[10]=IIC_ADD_read(MPU_6050_ADDR,MPU_GYRO_YOUTL_REG);
    mpu6050_buffer[11]=IIC_ADD_read(MPU_6050_ADDR,MPU_GYRO_YOUTH_REG);
    sensor.gyro.origin.y =	((int16_t)mpu6050_buffer[11]<<8)|mpu6050_buffer[10];

    mpu6050_buffer[12]=IIC_ADD_read(MPU_6050_ADDR,MPU_GYRO_ZOUTL_REG);
    mpu6050_buffer[13]=IIC_ADD_read(MPU_6050_ADDR,MPU_GYRO_ZOUTH_REG);
    sensor.gyro.origin.z =	((int16_t)mpu6050_buffer[13]<<8)|mpu6050_buffer[12];

}

void READ_MPU6050_MAG(void)
{ 
//    char BUF[6];
//    BUF[0]=IIC_ADD_read (MAG_ADDRESS,MAG_XOUT_L);
//    BUF[1]=IIC_ADD_read (MAG_ADDRESS,MAG_XOUT_H);
//    sensor.mag.origin.x =(BUF[1]<<8)|BUF[0];

//    BUF[2]=IIC_ADD_read(MAG_ADDRESS,MAG_YOUT_L);
//    BUF[3]=IIC_ADD_read(MAG_ADDRESS,MAG_YOUT_H);
//    sensor.mag.origin.y  =	(BUF[3]<<8)|BUF[2];
//                           //读取计算Y轴数据
//     
//    BUF[4]=IIC_ADD_read(MAG_ADDRESS,MAG_ZOUT_L);
//    BUF[5]=IIC_ADD_read(MAG_ADDRESS,MAG_ZOUT_H);
//    sensor.mag.origin.z  =	(BUF[5]<<8)|BUF[4];
// 					       //读取计算Z轴数据
}

void Get_6050_offest(void)
{
    u16 cnt=200;
    sensor.acc.temp.x=0;
	sensor.acc.temp.y=0;
	sensor.acc.temp.z=0;
	
	sensor.gyro.temp.x=0;
	sensor.gyro.temp.y=0;
	sensor.gyro.temp.z=0;
    
    while(cnt--)       //循环采集2000次   求平均
	 {
        READ_MPU6050_ACCEL();
        READ_MPU6050_GYRO();

        sensor.gyro.temp.x+= sensor.gyro.origin.x;
        sensor.gyro.temp.y+= sensor.gyro.origin.y;
        sensor.gyro.temp.z+= sensor.gyro.origin.z;

        sensor.acc.temp.x+= sensor.acc.origin.x;
        sensor.acc.temp.y+= sensor.acc.origin.y;
        sensor.acc.temp.z+= sensor.acc.origin.z;
    }
    cnt=200;
    sensor.gyro.quiet.x=(sensor.gyro.temp.x/cnt);
    sensor.gyro.quiet.y=sensor.gyro.temp.y/cnt;
    sensor.gyro.quiet.z=sensor.gyro.temp.z/cnt;

    sensor.acc.quiet.x=(sensor.acc.temp.x/cnt);
    sensor.acc.quiet.y=sensor.acc.temp.y/cnt;
    sensor.acc.quiet.z=sensor.acc.temp.z/cnt;
    
    Save_Acc_Gyro_offest();
}


//void Prepare_Data()
//{
//    READ_MPU9250_ACCEL();
//    READ_MPU9250_GYRO();
////    READ_MPU9250_MAG();
//    
//    sensor.acc.temp.x = sensor.acc.origin.x - sensor.acc.quiet.x;
//    sensor.acc.temp.y = sensor.acc.origin.y - sensor.acc.quiet.y;
//    sensor.acc.temp.z = sensor.acc.origin.z - sensor.acc.quiet.z;
//    
//    sensor.gyro.origin.x = (sensor.gyro.origin.x - sensor.gyro.quiet.x);
//    sensor.gyro.origin.y = (sensor.gyro.origin.y - sensor.gyro.quiet.y);
//    sensor.gyro.origin.z = (sensor.gyro.origin.z - sensor.gyro.quiet.z);
//    
//    sensor.gyro.averag.x = LPF_1st(sensor.gyro.averag.x ,sensor.gyro.origin.x,0.1f);
//    sensor.gyro.averag.y = LPF_1st(sensor.gyro.averag.y ,sensor.gyro.origin.y,0.1f);
//    sensor.gyro.averag.z = LPF_1st(sensor.gyro.averag.z ,sensor.gyro.origin.z,0.05f);
//    
//    sensor.acc.averag.x = KalmanFilter(sensor.acc.temp.x,KALMAN_Q,KALMAN_R,sensor.acc.averag.x,ACC_KALMAN_X);
//    sensor.acc.averag.y = KalmanFilter(sensor.acc.temp.y,KALMAN_Q,KALMAN_R,sensor.acc.averag.y,ACC_KALMAN_Y);
//    sensor.acc.averag.z = KalmanFilter(sensor.acc.temp.z,KALMAN_Q,KALMAN_R*2,sensor.acc.averag.z,ACC_KALMAN_Z);
//    
//    
//    sensor.acc.radian.x = sensor.acc.averag.x ;
//    sensor.acc.radian.y = sensor.acc.averag.y ;
//    sensor.acc.radian.z = sensor.acc.averag.z ;
//    sensor.gyro.radian.x = ((float)sensor.gyro.averag.x) *Gyro_Gr;
//    sensor.gyro.radian.y = ((float)sensor.gyro.averag.y) *Gyro_Gr;
//    sensor.gyro.radian.z = ((float)sensor.gyro.averag.z) *Gyro_Gr;
//}

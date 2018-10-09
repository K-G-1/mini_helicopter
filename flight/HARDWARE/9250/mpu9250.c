//#include "include.h"
#include "mpu9250.h"
#include "delay.h"
#include "IIC.h"
#include "imu.h"
#include "Algorithm_filter.h"


unsigned char TX_DATA[4];  	 //显示据缓存区
unsigned char BUF[10];       //接收数据缓存区
//char  test=0; 				 //IIC用到
short T_X,T_Y,T_Z,T_T;		 //X,Y,Z轴，温度



u8		 mpu6500_buffer[14];	
u8		 AK_8975_buffer[9];
struct _sensor sensor;

void mpu9250_init()
{
    u8 res;
    IIC_ADD_write(GYRO_ADDRESS,MPU6500_RA_PWR_MGMT_1,0x80); //Reset ing
	delay_ms(10);
	IIC_ADD_write(GYRO_ADDRESS,MPU6500_RA_SIGNAL_PATH_RESET,0x07);
	delay_ms(10);					 
    
    IIC_ADD_write(GYRO_ADDRESS,MPU6500_RA_PWR_MGMT_1,0x01); //Reset ing
    IIC_ADD_write(GYRO_ADDRESS,MPU6500_RA_PWR_MGMT_2,0x00); //Reset ing
    IIC_ADD_write(GYRO_ADDRESS,MPU6500_RA_CONFIG,0x03);  
	delay_ms(10);
    IIC_ADD_write(GYRO_ADDRESS,MPU6500_RA_SMPLRT_DIV,0x04);  
	IIC_ADD_write(GYRO_ADDRESS,MPU6500_RA_ACCEL_CONFIG,0x10);//加速度度最大量程 +-2G
	delay_ms(10);	
	IIC_ADD_write(GYRO_ADDRESS,MPU6500_RA_GYRO_CONFIG,0x10); //陀螺仪最大量程 +-1000度每秒
	delay_ms(10);

    
    IIC_ADD_write(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
    delay_ms(100);	
    IIC_ADD_write(MAG_ADDRESS,0x0A,0x01);
    delay_ms(100);	
    
    res=IIC_ADD_read(GYRO_ADDRESS,MPU6500_RA_WHO_AM_I);
    
    while(res!=0x71);
    
    READ_MPU9250_MAG();
    delay_ms(100);
    READ_MPU9250_MAG();
}

//******读取MPU9250数据****************************************
void READ_9250()
{
    IIC_Read_MultiBytes(GYRO_ADDRESS,MPU6500_RA_ACCEL_XOUT_H,14,mpu6500_buffer);
    sensor.acc.origin.x = ((((int16_t)mpu6500_buffer[0]) << 8) | mpu6500_buffer[1]);
	sensor.acc.origin.y = ((((int16_t)mpu6500_buffer[2]) << 8) | mpu6500_buffer[3]);
	sensor.acc.origin.z = ((((int16_t)mpu6500_buffer[4]) << 8) | mpu6500_buffer[5]) ;
    sensor.gyro.origin.x = ((((int16_t)mpu6500_buffer[8]) << 8) | mpu6500_buffer[9]);
	sensor.gyro.origin.y = ((((int16_t)mpu6500_buffer[10]) << 8)| mpu6500_buffer[11]);
	sensor.gyro.origin.z = ((((int16_t)mpu6500_buffer[12]) << 8)| mpu6500_buffer[13]);
    
////    IIC_ADD_write(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 	
//    
////    delay_ms(20);	
//    AK_8975_buffer[8] = IIC_ADD_read(MAG_ADDRESS,0x02); 
//    if(AK_8975_buffer[8] == 1)
//    {
//    IIC_Read_MultiBytes(MAG_ADDRESS,0x03,6,AK_8975_buffer);
////    sensor.mag.origin.x = ((((int16_t)AK_8975_buffer[0]) << 8) | AK_8975_buffer[1]);
////    sensor.mag.origin.y = ((((int16_t)AK_8975_buffer[2]) << 8) | AK_8975_buffer[3]);
////    sensor.mag.origin.z = ((((int16_t)AK_8975_buffer[4]) << 8) | AK_8975_buffer[5]);
//    sensor.mag.origin.x = ((((int16_t)AK_8975_buffer[1]) << 8) | AK_8975_buffer[0]);
//    sensor.mag.origin.y = ((((int16_t)AK_8975_buffer[3]) << 8) | AK_8975_buffer[2]);
//    sensor.mag.origin.z = ((((int16_t)AK_8975_buffer[5]) << 8) | AK_8975_buffer[4]);
//        IIC_ADD_write(MAG_ADDRESS,0x0A,0x01);
//    }
}

void MPU6500_Dataanl(T_int16_xyz *data_tempacc,T_int16_xyz *data_tempgyr)
{
    IIC_Read_MultiBytes(GYRO_ADDRESS,MPU6500_RA_ACCEL_XOUT_H,14,mpu6500_buffer);
    
    data_tempacc->x = ((((int16_t)mpu6500_buffer[0]) << 8) | mpu6500_buffer[1]);
	data_tempacc->y = ((((int16_t)mpu6500_buffer[2]) << 8) | mpu6500_buffer[3]);
	data_tempacc->z = ((((int16_t)mpu6500_buffer[4]) << 8) | mpu6500_buffer[5]) ;
    data_tempgyr->x = ((((int16_t)mpu6500_buffer[8]) << 8) | mpu6500_buffer[9]);
	data_tempgyr->y = ((((int16_t)mpu6500_buffer[10]) << 8)| mpu6500_buffer[11]);
	data_tempgyr->z = ((((int16_t)mpu6500_buffer[12]) << 8)| mpu6500_buffer[13]);
}
void READ_MPU9250_ACCEL(void)
{ 

    mpu6500_buffer[0]=IIC_ADD_read(ACCEL_ADDRESS,MPU6500_RA_ACCEL_XOUT_L); 
    mpu6500_buffer[1]=IIC_ADD_read(ACCEL_ADDRESS,MPU6500_RA_ACCEL_XOUT_H);
    sensor.acc.origin.x=	((int16_t)mpu6500_buffer[1]<<8)|mpu6500_buffer[0];             //读取计算X轴数据

    mpu6500_buffer[2]=IIC_ADD_read(ACCEL_ADDRESS,MPU6500_RA_ACCEL_YOUT_L);
    mpu6500_buffer[3]=IIC_ADD_read(ACCEL_ADDRESS,MPU6500_RA_ACCEL_YOUT_H);
    sensor.acc.origin.y =	((int16_t)mpu6500_buffer[3]<<8)|mpu6500_buffer[2];             //读取计算Y轴数据
    
    mpu6500_buffer[4]=IIC_ADD_read(ACCEL_ADDRESS,MPU6500_RA_ACCEL_ZOUT_L);
    mpu6500_buffer[5]=IIC_ADD_read(ACCEL_ADDRESS,MPU6500_RA_ACCEL_ZOUT_H);
    sensor.acc.origin.z =	((int16_t)mpu6500_buffer[5]<<8)|mpu6500_buffer[4];             //读取计算Z轴数据
 
    
}

void READ_MPU9250_GYRO(void)
{ 
    
    mpu6500_buffer[8]=IIC_ADD_read(GYRO_ADDRESS,MPU6500_RA_GYRO_XOUT_L); 
    mpu6500_buffer[9]=IIC_ADD_read(GYRO_ADDRESS,MPU6500_RA_GYRO_XOUT_H);
    sensor.gyro.origin.x =	((int16_t)mpu6500_buffer[9]<<8)|mpu6500_buffer[8];

    mpu6500_buffer[10]=IIC_ADD_read(GYRO_ADDRESS,MPU6500_RA_GYRO_YOUT_L);
    mpu6500_buffer[11]=IIC_ADD_read(GYRO_ADDRESS,MPU6500_RA_GYRO_YOUT_H);
    sensor.gyro.origin.y =	((int16_t)mpu6500_buffer[11]<<8)|mpu6500_buffer[10];

    mpu6500_buffer[12]=IIC_ADD_read(GYRO_ADDRESS,MPU6500_RA_GYRO_ZOUT_L);
    mpu6500_buffer[13]=IIC_ADD_read(GYRO_ADDRESS,MPU6500_RA_GYRO_ZOUT_H);
    sensor.gyro.origin.z =	((int16_t)mpu6500_buffer[13]<<8)|mpu6500_buffer[12];

        
    if(sensor.gyro.origin.y==0x00ff)
       sensor.gyro.origin.y = sensor.gyro.averag.y; 
    if(sensor.gyro.origin.z==0x00ff)
       sensor.gyro.origin.z = sensor.gyro.averag.z; 
}

void READ_MPU9250_MAG(void)
{ 

//    delay_ms(10);
    
    
    IIC_Read_MultiBytes(MAG_ADDRESS,0x00,9,AK_8975_buffer);
//    sensor.mag.origin.x = ((((int16_t)AK_8975_buffer[0]) << 8) | AK_8975_buffer[1]);
//    sensor.mag.origin.y = ((((int16_t)AK_8975_buffer[2]) << 8) | AK_8975_buffer[3]);
//    sensor.mag.origin.z = ((((int16_t)AK_8975_buffer[4]) << 8) | AK_8975_buffer[5]);
    sensor.mag.origin.x = ((((int16_t)AK_8975_buffer[1]) << 8) | AK_8975_buffer[0]);
    sensor.mag.origin.y = ((((int16_t)AK_8975_buffer[3]) << 8) | AK_8975_buffer[2]);
    sensor.mag.origin.z = ((((int16_t)AK_8975_buffer[5]) << 8) | AK_8975_buffer[4]);
    IIC_ADD_write(MAG_ADDRESS,0x0A,0x01);

}

void Get_offest(void)
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
        READ_9250();
         
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
    
    
}


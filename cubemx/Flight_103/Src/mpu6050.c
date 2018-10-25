
#include "mpu6050.h"
#include "I2C.h"

uint8_t		 mpu6050_buffer[14];	
uint8_t res;

struct _sensor sensor;


uint8_t mpu6050_init(void)
{
    
//    IIC_ADD_write(MPU_6050_ADDR,MPU_PWR_MGMT1_REG,0x00);
//  IIC_ADD_write(MPU_6050_ADDR,MPU_PWR_MGMT1_REG,0x03); 
//  IIC_ADD_write(MPU_6050_ADDR,MPU_SAMPLE_RATE_REG,0x00);
//  IIC_ADD_write(MPU_6050_ADDR,MPU_CFG_REG,0x04);  
//  IIC_ADD_write(MPU_6050_ADDR,MPU_ACCEL_CFG_REG,0x10);
//  IIC_ADD_write(MPU_6050_ADDR,MPU_GYRO_CFG_REG,0x18); 
//  res=IIC_ADD_read(MPU_6050_ADDR,MPU_DEVICE_ID_REG);
  
  
    res = 0x00;    
    HAL_I2C_Mem_Write(&hi2c1,MPU_6050_ADDR,MPU_PWR_MGMT1_REG,I2C_MEMADD_SIZE_8BIT,&res,1,0x10);//Reset ing
    res = 0x03; 
    HAL_I2C_Mem_Write(&hi2c1,MPU_6050_ADDR,MPU_PWR_MGMT1_REG,I2C_MEMADD_SIZE_8BIT,&res,1,0x10);//Reset ing	
    res = 0x00; 
    HAL_I2C_Mem_Write(&hi2c1,MPU_6050_ADDR,MPU_SAMPLE_RATE_REG,I2C_MEMADD_SIZE_8BIT,&res,1,0x10);        
    res = 0x04; 
    HAL_I2C_Mem_Write(&hi2c1,MPU_6050_ADDR,MPU_CFG_REG,I2C_MEMADD_SIZE_8BIT,&res,1,0x10);	
    res = 0x10; 
    HAL_I2C_Mem_Write(&hi2c1,MPU_6050_ADDR,MPU_ACCEL_CFG_REG,I2C_MEMADD_SIZE_8BIT,&res,1,0x10);//加速度度最大量程 +-8G	
    res = 0x18; 
    HAL_I2C_Mem_Write(&hi2c1,MPU_6050_ADDR,MPU_GYRO_CFG_REG,I2C_MEMADD_SIZE_8BIT,&res,1,0x10);//陀螺仪最大量程 +-2000度每秒
    HAL_I2C_Mem_Read(&hi2c1,MPU_6050_ADDR,MPU_DEVICE_ID_REG,I2C_MEMADD_SIZE_8BIT,&res,1,0x10);
       HAL_Delay(100);
    if(res==0x68)//器件ID正确
	{
		return 0;
 	}else 
	{
		return 1;
	}
}

//******读取MPU9250数据****************************************

void READ_MPU6050(void)
{
//    HAL_I2C_Mem_Read(&hi2c1,MPU_6050_ADDR,MPU_ACCEL_XOUTH_REG,I2C_MEMADD_SIZE_8BIT,mpu6050_buffer,14,0x80);
  sensor.acc.origin.x=	((int16_t)mpu6050_buffer[0]<<8)|mpu6050_buffer[1];             //读取计算X轴数据
  sensor.acc.origin.y =	((int16_t)mpu6050_buffer[2]<<8)|mpu6050_buffer[3];             //读取计算Y轴数据
  sensor.acc.origin.z =	((int16_t)mpu6050_buffer[4]<<8)|mpu6050_buffer[5];             //读取计算Z轴数据
  
  sensor.gyro.origin.x =	((int16_t)mpu6050_buffer[8]<<8)|mpu6050_buffer[9];
  sensor.gyro.origin.y =	((int16_t)mpu6050_buffer[10]<<8)|mpu6050_buffer[11];
  sensor.gyro.origin.z =	((int16_t)mpu6050_buffer[12]<<8)|mpu6050_buffer[13];
}

void READ_MPU6050_ACCEL(void)
{ 
//    HAL_I2C_Mem_Read(&hi2c1,MPU_6050_ADDR,MPU_ACCEL_XOUTH_REG,I2C_MEMADD_SIZE_8BIT,mpu6050_buffer,14,0x10);
//    mpu6050_buffer[0]=IIC_ADD_read(MPU_6050_ADDR,MPU_ACCEL_XOUTL_REG); 
//    mpu6050_buffer[1]=IIC_ADD_read(MPU_6050_ADDR,MPU_ACCEL_XOUTH_REG);
//    sensor.acc.origin.x=	((int16_t)mpu6050_buffer[1]<<8)|mpu6050_buffer[0];             //读取计算X轴数据

//    mpu6050_buffer[2]=IIC_ADD_read(MPU_6050_ADDR,MPU_ACCEL_YOUTL_REG);
//    mpu6050_buffer[3]=IIC_ADD_read(MPU_6050_ADDR,MPU_ACCEL_YOUTH_REG);
//    sensor.acc.origin.y =	((int16_t)mpu6050_buffer[3]<<8)|mpu6050_buffer[2];             //读取计算Y轴数据
//    
//    mpu6050_buffer[4]=IIC_ADD_read(MPU_6050_ADDR,MPU_ACCEL_ZOUTL_REG);
//    mpu6050_buffer[5]=IIC_ADD_read(MPU_6050_ADDR,MPU_ACCEL_ZOUTH_REG);
//    sensor.acc.origin.z =	((int16_t)mpu6050_buffer[5]<<8)|mpu6050_buffer[4];             //读取计算Z轴数据
 
}

void READ_MPU6050_GYRO(void)
{ 
    
//    mpu6050_buffer[8]=IIC_ADD_read(MPU_6050_ADDR,MPU_GYRO_XOUTL_REG); 
//    mpu6050_buffer[9]=IIC_ADD_read(MPU_6050_ADDR,MPU_GYRO_XOUTH_REG);
//    sensor.gyro.origin.x =	((int16_t)mpu6050_buffer[9]<<8)|mpu6050_buffer[8];

//    mpu6050_buffer[10]=IIC_ADD_read(MPU_6050_ADDR,MPU_GYRO_YOUTL_REG);
//    mpu6050_buffer[11]=IIC_ADD_read(MPU_6050_ADDR,MPU_GYRO_YOUTH_REG);
//    sensor.gyro.origin.y =	((int16_t)mpu6050_buffer[11]<<8)|mpu6050_buffer[10];

//    mpu6050_buffer[12]=IIC_ADD_read(MPU_6050_ADDR,MPU_GYRO_ZOUTL_REG);
//    mpu6050_buffer[13]=IIC_ADD_read(MPU_6050_ADDR,MPU_GYRO_ZOUTH_REG);
//    sensor.gyro.origin.z =	((int16_t)mpu6050_buffer[13]<<8)|mpu6050_buffer[12];

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
    uint16_t cnt=200;
    sensor.acc.temp.x=0;
	sensor.acc.temp.y=0;
	sensor.acc.temp.z=0;
	
	sensor.gyro.temp.x=0;
	sensor.gyro.temp.y=0;
	sensor.gyro.temp.z=0;
    
    while(cnt--)       //循环采集2000次   求平均
	 {
        READ_MPU6050();

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



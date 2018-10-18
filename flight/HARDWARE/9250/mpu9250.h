#ifndef __mpu9250_H__
#define __mpu9250_H__
#include "sys.h"
#include "imu.h"

#define USE_IMU_DEVICE 1	//0:9250 1:6050

#define RtA 		57.324841f		//  180/3.1415  弧度转化为角度		
#define AtR    	0.0174533f		//  1/RtA             RtA倒数		
#define Acc_G 	0.0011963f		//  1/32768/4/9.8     加速度量程为4G		int型1/(2^15)/4g
#define Gyro_G 	0.03051756f	//  1/32768/1000      陀螺仪初始化的量程为 +—1000			1/(2^15)/4g单位为度/秒
#define Gyro_Gr	0.0005327f  //  1/32768/1000/57.3 将上面的单位转化为弧度每秒
#define Gyro_G_x  0.06103f       //0.061036 //   4000/65536  +-2000



// 定义MPU9250内部地址
//****************************************
#define MPU6500_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6500_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6500_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6500_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6500_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6500_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6500_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6500_RA_XA_OFFS_L_TC     0x07
#define MPU6500_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6500_RA_YA_OFFS_L_TC     0x09
#define MPU6500_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6500_RA_ZA_OFFS_L_TC     0x0B
#define MPU6500_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6500_RA_XG_OFFS_USRL     0x14
#define MPU6500_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6500_RA_YG_OFFS_USRL     0x16
#define MPU6500_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6500_RA_ZG_OFFS_USRL     0x18
#define MPU6500_RA_SMPLRT_DIV       0x19
#define MPU6500_RA_CONFIG           0x1A
#define MPU6500_RA_GYRO_CONFIG      0x1B
#define MPU6500_RA_ACCEL_CONFIG     0x1C
#define MPU6500_RA_FF_THR           0x1D
#define MPU6500_RA_FF_DUR           0x1E
#define MPU6500_RA_MOT_THR          0x1F
#define MPU6500_RA_MOT_DUR          0x20
#define MPU6500_RA_ZRMOT_THR        0x21
#define MPU6500_RA_ZRMOT_DUR        0x22
#define MPU6500_RA_FIFO_EN          0x23
#define MPU6500_RA_I2C_MST_CTRL     0x24
#define MPU6500_RA_I2C_SLV0_ADDR    0x25
#define MPU6500_RA_I2C_SLV0_REG     0x26
#define MPU6500_RA_I2C_SLV0_CTRL    0x27
#define MPU6500_RA_I2C_SLV1_ADDR    0x28
#define MPU6500_RA_I2C_SLV1_REG     0x29
#define MPU6500_RA_I2C_SLV1_CTRL    0x2A
#define MPU6500_RA_I2C_SLV2_ADDR    0x2B
#define MPU6500_RA_I2C_SLV2_REG     0x2C
#define MPU6500_RA_I2C_SLV2_CTRL    0x2D
#define MPU6500_RA_I2C_SLV3_ADDR    0x2E
#define MPU6500_RA_I2C_SLV3_REG     0x2F
#define MPU6500_RA_I2C_SLV3_CTRL    0x30
#define MPU6500_RA_I2C_SLV4_ADDR    0x31
#define MPU6500_RA_I2C_SLV4_REG     0x32
#define MPU6500_RA_I2C_SLV4_DO      0x33
#define MPU6500_RA_I2C_SLV4_CTRL    0x34
#define MPU6500_RA_I2C_SLV4_DI      0x35
#define MPU6500_RA_I2C_MST_STATUS   0x36
#define MPU6500_RA_INT_PIN_CFG      0x37
#define MPU6500_RA_INT_ENABLE       0x38
#define MPU6500_RA_DMP_INT_STATUS   0x39
#define MPU6500_RA_INT_STATUS       0x3A
#define MPU6500_RA_ACCEL_XOUT_H     0x3B
#define MPU6500_RA_ACCEL_XOUT_L     0x3C
#define MPU6500_RA_ACCEL_YOUT_H     0x3D
#define MPU6500_RA_ACCEL_YOUT_L     0x3E
#define MPU6500_RA_ACCEL_ZOUT_H     0x3F
#define MPU6500_RA_ACCEL_ZOUT_L     0x40
#define MPU6500_RA_TEMP_OUT_H       0x41
#define MPU6500_RA_TEMP_OUT_L       0x42
#define MPU6500_RA_GYRO_XOUT_H      0x43
#define MPU6500_RA_GYRO_XOUT_L      0x44
#define MPU6500_RA_GYRO_YOUT_H      0x45
#define MPU6500_RA_GYRO_YOUT_L      0x46
#define MPU6500_RA_GYRO_ZOUT_H      0x47
#define MPU6500_RA_GYRO_ZOUT_L      0x48
#define MPU6500_RA_EXT_SENS_DATA_00 0x49
#define MPU6500_RA_EXT_SENS_DATA_01 0x4A
#define MPU6500_RA_EXT_SENS_DATA_02 0x4B
#define MPU6500_RA_EXT_SENS_DATA_03 0x4C
#define MPU6500_RA_EXT_SENS_DATA_04 0x4D
#define MPU6500_RA_EXT_SENS_DATA_05 0x4E
#define MPU6500_RA_EXT_SENS_DATA_06 0x4F
#define MPU6500_RA_EXT_SENS_DATA_07 0x50
#define MPU6500_RA_EXT_SENS_DATA_08 0x51
#define MPU6500_RA_EXT_SENS_DATA_09 0x52
#define MPU6500_RA_EXT_SENS_DATA_10 0x53
#define MPU6500_RA_EXT_SENS_DATA_11 0x54
#define MPU6500_RA_EXT_SENS_DATA_12 0x55
#define MPU6500_RA_EXT_SENS_DATA_13 0x56
#define MPU6500_RA_EXT_SENS_DATA_14 0x57
#define MPU6500_RA_EXT_SENS_DATA_15 0x58
#define MPU6500_RA_EXT_SENS_DATA_16 0x59
#define MPU6500_RA_EXT_SENS_DATA_17 0x5A
#define MPU6500_RA_EXT_SENS_DATA_18 0x5B
#define MPU6500_RA_EXT_SENS_DATA_19 0x5C
#define MPU6500_RA_EXT_SENS_DATA_20 0x5D
#define MPU6500_RA_EXT_SENS_DATA_21 0x5E
#define MPU6500_RA_EXT_SENS_DATA_22 0x5F
#define MPU6500_RA_EXT_SENS_DATA_23 0x60
#define MPU6500_RA_MOT_DETECT_STATUS    0x61
#define MPU6500_RA_I2C_SLV0_DO      0x63
#define MPU6500_RA_I2C_SLV1_DO      0x64
#define MPU6500_RA_I2C_SLV2_DO      0x65
#define MPU6500_RA_I2C_SLV3_DO      0x66
#define MPU6500_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6500_RA_SIGNAL_PATH_RESET    0x68
#define MPU6500_RA_MOT_DETECT_CTRL      0x69
#define MPU6500_RA_USER_CTRL        0x6A
#define MPU6500_RA_PWR_MGMT_1       0x6B
#define MPU6500_RA_PWR_MGMT_2       0x6C
#define MPU6500_RA_BANK_SEL         0x6D
#define MPU6500_RA_MEM_START_ADDR   0x6E
#define MPU6500_RA_MEM_R_W          0x6F
#define MPU6500_RA_DMP_CFG_1        0x70
#define MPU6500_RA_DMP_CFG_2        0x71
#define MPU6500_RA_FIFO_COUNTH      0x72
#define MPU6500_RA_FIFO_COUNTL      0x73
#define MPU6500_RA_FIFO_R_W         0x74
#define MPU6500_RA_WHO_AM_I         0x75

#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08
//****************************

#define	GYRO_ADDRESS   0xD2	  //陀螺地址
#define MAG_ADDRESS    0x18   //磁场地址
#define MAG_ADDRESS_H    0x1A   //磁场地址
#define ACCEL_ADDRESS  0xD2 
//*********************************
typedef struct{
				float X;
				float Y;
				float Z;}FLOAT_XYZ;

struct _float{
	      float x;
				float y;
				float z;};

struct _short{
	      short x;
				short y;
				short z;};				
struct _int16{
         int16_t x;
	     int16_t y;
	     int16_t z;};	

struct _int32{
       int32_t x;
	     int32_t y;
	     int32_t z;};	


struct _uint32{
       uint32_t x;
	     uint32_t y;
	     uint32_t z;};	

struct _long
      {
			long x;
			long y;
			long z;
       };


struct _trans{
        struct _int16 origin;  //原始值
        struct _int16 averag;  //平均值
        struct _int16 histor;  //历史值
        struct _int16 quiet;   //静态值
        struct _float radian;  //弧度值
        struct  _long temp;
        struct _int16 sand;
        
        u8 CALIBRATE;
    
          };

struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
    struct _trans mag;
              };

extern struct _sensor sensor;
extern u8		 mpu6500_buffer[14];	

//
u8 mpu9250_init(void);
void READ_9250(void);
void Get_offest(void);
void READ_MPU9250_ACCEL(void);
void READ_MPU9250_GYRO(void);
void READ_MPU9250_MAG(void);
void Prepare_Data(void);   
void MPU6500_Dataanl(T_int16_xyz *data_tempacc,T_int16_xyz *data_tempgyr);              
#endif


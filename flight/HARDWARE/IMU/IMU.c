#include "IMU.h"
#include "math.h"
#include "mpu9250.h"
#include "mpu6050.h"
#include "Algorithm_math.h"
#include "Algorithm_filter.h"

struct _angle angle;
struct IMU_DATA imu_data;
#define KALMAN_Q        0.012
#define KALMAN_R        81.0000
double ACC_KALMAN_X  =1;
double ACC_KALMAN_Y  =1;
double ACC_KALMAN_Z  =1;

#define FILTER_NUM 	20

void Prepare_Data()
{
    static uint8_t 	filter_cnt=0;
	static int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;

    sensor.acc.origin.x = (imu_data.mpu6500_dataacc1.x + imu_data.mpu6500_dataacc2.x)/2;
    sensor.acc.origin.y = (imu_data.mpu6500_dataacc1.y + imu_data.mpu6500_dataacc2.y)/2;
    sensor.acc.origin.z = (imu_data.mpu6500_dataacc1.z + imu_data.mpu6500_dataacc2.z)/2;
    sensor.gyro.origin.x = (imu_data.mpu6500_datagyr1.x + imu_data.mpu6500_datagyr2.x)/2;
    sensor.gyro.origin.y = (imu_data.mpu6500_datagyr1.y + imu_data.mpu6500_datagyr2.y)/2;
    sensor.gyro.origin.z = (imu_data.mpu6500_datagyr1.z + imu_data.mpu6500_datagyr2.z)/2;
  
    sensor.acc.temp.x = sensor.acc.origin.x - sensor.acc.quiet.x;
    sensor.acc.temp.y = sensor.acc.origin.y - sensor.acc.quiet.y;
    sensor.acc.temp.z = sensor.acc.origin.z ;
    
    sensor.gyro.temp.x = (sensor.gyro.origin.x - sensor.gyro.quiet.x);
    sensor.gyro.temp.y = (sensor.gyro.origin.y - sensor.gyro.quiet.y);
    sensor.gyro.temp.z = (sensor.gyro.origin.z - sensor.gyro.quiet.z);
////////    
    sensor.gyro.averag.x = LPF_1st(sensor.gyro.averag.x ,sensor.gyro.temp.x,0.1f);
    sensor.gyro.averag.y = LPF_1st(sensor.gyro.averag.y ,sensor.gyro.temp.y,0.1f);
    sensor.gyro.averag.z = LPF_1st(sensor.gyro.averag.z ,sensor.gyro.temp.z,0.05f);
    
    sensor.gyro.radian.x = ((float)sensor.gyro.averag.x) *Gyro_Gr;
    sensor.gyro.radian.y = ((float)sensor.gyro.averag.y) *Gyro_Gr;
    sensor.gyro.radian.z = ((float)sensor.gyro.averag.z) *Gyro_Gr;
////////    
    ACC_X_BUF[filter_cnt] = sensor.acc.temp.x;
	ACC_Y_BUF[filter_cnt] = sensor.acc.temp.y ;
	ACC_Z_BUF[filter_cnt] = sensor.acc.temp.z;
    
    for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
    filter_cnt++;
    
    sensor.acc.averag.x = temp1 /FILTER_NUM;
    sensor.acc.averag.y = temp2 /FILTER_NUM;
    sensor.acc.averag.z = temp3 /FILTER_NUM;
    
    if(filter_cnt==FILTER_NUM)	
        filter_cnt=0;
    
    sensor.acc.radian.x = sensor.acc.averag.x ;
    sensor.acc.radian.y = sensor.acc.averag.y ;
    sensor.acc.radian.z = sensor.acc.averag.z ;
    
}


float Kp= 2.0f;
float Ki= 0.002f;
//#define Kp 0.5f                        // 比例增益支配收敛率accelerometer/magnetometer  
//#define Ki 0.002f                     // 积分增益支配执政速率陀螺仪的衔接gyroscopeases  //KP,KI需要调的
#define halfT 0.001f                 // 采样周期的一半  本程序 2.5MS 采集一次  所以 halfT是1.25MS

/**************************************
 * 函数名：Get_Attitude
 * 描述  ：得到当前姿态
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 *************************************/
void Get_Attitude(void)
{
//	Prepare_Data();
	
	IMUupdate(  sensor.gyro.radian.x,
                sensor.gyro.radian.y,
                sensor.gyro.radian.z,
                sensor.acc.averag.x,
                sensor.acc.averag.y,
                sensor.acc.averag.z,
                sensor.mag.origin.x,
                sensor.mag.origin.y,
                sensor.mag.origin.z
            );	
//    IMUupdate1(  sensor.gyro.radian.x,
//                sensor.gyro.radian.y,
//                sensor.gyro.radian.z,
//                sensor.acc.radian.x,
//                sensor.acc.radian.y,
//                sensor.acc.radian.z
//            );	

}

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    //四元数的元素，代表估计方向
float exInt = 0, eyInt = 0, ezInt = 0;    // 按比例缩小积分误差
float q0_yaw = 1, q1_yaw = 0, q2_yaw = 0, q3_yaw = 0;    //弥补Mahony算法在无地磁情况解算Yaw轴满足不了大扰动要求的现象

void IMUupdate1(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
    float q1q3 = q1 * q3;
    float q2q2 = q2	* q2;
    float q2q3 = q2	*	q3;
    float q3q3 = q3	*	q3;	
    float vx, vy, vz;
    float ex, ey, ez;

    float	q0_yawq0_yaw = q0_yaw * q0_yaw;
    float	q1_yawq1_yaw = q1_yaw * q1_yaw;
    float	q2_yawq2_yaw = q2_yaw * q2_yaw;
    float	q3_yawq3_yaw = q3_yaw * q3_yaw;
    float	q1_yawq2_yaw = q1_yaw * q2_yaw;
    float	q0_yawq3_yaw = q0_yaw * q3_yaw;

    //**************************Yaw轴计算******************************

    //Yaw轴四元素的微分方程
    q0_yaw = q0_yaw + (-q1_yaw * gx - q2_yaw * gy - q3_yaw * gz) * halfT;
    q1_yaw = q1_yaw + (q0_yaw * gx + q2_yaw * gz - q3_yaw * gy) * halfT;
    q2_yaw = q2_yaw + (q0_yaw * gy - q1_yaw * gz + q3_yaw * gx) * halfT;
    q3_yaw = q3_yaw + (q0_yaw * gz + q1_yaw * gy - q2_yaw * gx) * halfT;

    //规范化Yaw轴四元数
    norm = sqrt(q0_yawq0_yaw + q1_yawq1_yaw + q2_yawq2_yaw + q3_yawq3_yaw);
    q0_yaw = q0_yaw / norm;
    q1_yaw = q1_yaw / norm;
    q2_yaw = q2_yaw / norm;
    q3_yaw = q3_yaw / norm;


    if(ax * ay * az	== 0)
    return ;

    //规范化加速度计值
    norm = sqrt(ax * ax + ay * ay + az * az); 
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;

    //估计重力方向和流量/变迁
    vx = 2 * (q1q3 - q0q2);											
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;

    //向量外积再相减得到差分就是误差
    ex = (ay * vz - az * vy) ;      
    ey = (az * vx - ax * vz) ;
    ez = (ax * vy - ay * vx) ;

    //对误差进行PI计算
    exInt = exInt + ex * Ki;			
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    //校正陀螺仪
    gx = gx + Kp * ex + exInt;					
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;			
        
    //四元素的微分方程
    q0 = q0 + (-q1 * gx - q2	*	gy - q3	*	gz)	*	halfT;
    q1 = q1 + (q0	*	gx + q2	*	gz - q3	*	gy)	*	halfT;
    q2 = q2 + (q0	*	gy - q1	*	gz + q3	*	gx)	*	halfT;
    q3 = q3 + (q0	*	gz + q1	*	gy - q2	*	gx)	*	halfT;

    //规范化Pitch、Roll轴四元数
    norm = sqrt(q0q0 + q1q1 + q2q2 + q3q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    angle.yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* RtA;
    //	angle.yaw=0;
    angle.roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) *RtA; // roll
    angle.pitch = asin(-2*q1*q3 + 2*q0*q2) *RtA; // pitch

}



void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
  double norm,tempyaw;
  double hx, hy, hz, bx, bz;
  double vx, vy, vz, wx, wy, wz;
  double ex, ey, ez;
 // float tempq0,tempq1,tempq2,tempq3;
  
//辅助变量以减少重复操作,先把要用到的值算好
  double q0q0 = q0*q0;
  double q0q1 = q0*q1;
  double q0q2 = q0*q2;
  double q0q3 = q0*q3;
  double q1q1 = q1*q1;
  double q1q2 = q1*q2;
  double q1q3 = q1*q3;
  double q2q2 = q2*q2;   
  double q2q3 = q2*q3;
  double q3q3 = q3*q3; 
	//测量正常化
	if(ax*ay*az==0)
 		return;
		
  norm = Q_rsqrt(ax*ax + ay*ay + az*az); 
//加速度计数据归一化，把加速度计的三维向量转化为单位向量，应为是单位矢量到参考系的投影，所以要把acc单位化
  ax = ax * norm;
	//其实归一化改变的只是这三个向量的长度
  ay = ay * norm;
	//改变了相同的倍数，方向并没有改变，也是为了与单位向量对应
  az = az * norm;//四元数对应
	
	norm = Q_rsqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;
	//计算参考磁通方向
	hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;   
	//估计方向的重力和磁通(V和W)  （是带有误差的姿态，后面需要用到卡尔曼或互补滤波将误差滤掉）
  vx = 2*(q1q3 - q0q2);//参考系Z与载体系x轴之间的方向余弦向量
  vy = 2*(q0q1 + q2q3);//参考系Z与载体系y轴之间的方向余弦向量
  vz = q0q0 - q1q1 - q2q2 + q3q3;//参考系Z与载体系z轴之间方向余弦向量
  wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2); 
	// 错误是跨产品的总和之间的参考方向的领域和方向测量传感器
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  exInt = exInt + VariableParameter(ex) * ex * Ki;								  //对误差进行积分，会造成误差累积
  eyInt = eyInt + VariableParameter(ey) * ey * Ki;
  ezInt = ezInt + VariableParameter(ez) * ez * Ki;
// adjusted gyroscope measurements

  gx = gx + Kp *  VariableParameter(ex) * ex + exInt;	
	gy = gy + Kp *  VariableParameter(ey) * ey + eyInt;	
	gz = gz + Kp *  VariableParameter(ez) * ez + ezInt;	
//积分误差比例增益
  exInt = exInt + halfT * ex * Ki;//对误差进行积分						
  eyInt = eyInt + halfT * ey * Ki;
  ezInt = ezInt + halfT * ez * Ki;
//调整后的陀螺仪测量
  gx = gx + Kp  * ex + exInt;	//将误差积分后补偿到陀螺仪，即补偿零点漂移
	gy = gy + Kp  * ey + eyInt;	//修正陀螺仪输出
	gz = gz + Kp  * ez + ezInt;	//这里的gz由于没有观测者进行矫正，所以表现出来的就是积分的自增或自减

  
//整合四元数率和正常化	
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;//四元数的微分方程，halfT即姿态解算周期的一半
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // 正常化四元数，计算欧拉角时必须对四元数进行规范化处理
  norm = Q_rsqrt(q0q0 + q1q1 + q2q2 + q3q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
//输入参数为加速度XYZ数据，磁力计XYZ为原始数据，陀螺仪数据必须转化为弧度制，由计算的四元数计算欧拉角

	angle.yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* RtA ;         //*RtA转化为角度
//	if(tempyaw>0)
//		angle.yaw=tempyaw;
//	else
//		angle.yaw=360+tempyaw;
  angle.roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) *RtA; // roll
	angle.pitch = asin(-2*q1*q3 + 2*q0*q2) *RtA; // pitch

	


}












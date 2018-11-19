/*******************************************************************************************
										    声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现其他
	
不可估量的BUG，天际智联不负任何责任。请勿商用！

* 程序版本：V1.01
* 程序日期：2018-8-18
* 程序作者：愤怒的小孩
* 版权所有：西安天际智联信息技术有限公司
*******************************************************************************************/
#include "main.h"


  
void System_Init(void)
{
	NvicConfig(); //系统中断优先级管理
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	LED_Init();	//用户指示灯初始化
	Delay_Init(); //系统延时初始化
	USART_init(115200);	//调试串口初始化
	IIC_GPIO_Init(); //模拟IIC初始化
	TIM_Init();	//系统时基初始化
	Exit_Init(); //外部中断初始化
	SI24R1_Init(); //SI24R1(2.4G)初始化（红）
  SI24R1_Config();
	MPU6050_Init(); //MPU6050初始化（绿） 
//	bmp280Init(); //FBM320初始化(气压计蓝) 
	MOTOR_Init(); //电机输出初始化
	BATT_Init(); //电池电压检测初始化
	PID_ReadFlash(); //Flash中的数据读取
	PidParameter_init(); //PID参数初始化

	
//	printf("System Init Finish\n");
}

void Task_Schedule(void)
{
  uint8_t sta;
	if(ANO_Scan) //500Hz
		{
			ANO_Scan = 0;
			ANO_DT_Data_Exchange(); //更新数据到上位机
		}
		if(IMU_Scan) //100Hz
		{      

			IMU_Scan  = 0;
			Prepare_Data(); //获取姿态解算所需数据
			IMUupdate(&Gyr_rad,&Acc_filt,&Att_Angle); //四元数姿态解算
			Control(&Att_Angle,&Gyr_rad,&RC_Control,Airplane_Enable); //姿态控制
//			bmp280GetData(&Bmp280.bmp280_temp,&Bmp280.bmp280_press,&Bmp280.Altitude);
		}
		if(LED_Scan) //10Hz
		{
			LED_Scan = 0;
			LED_Run();
			Deblocking();
//			if(!Airplane_Enable&&Run_flag)
//			{
//				RGB_LED_Runing(); //飞机上锁状态灯
//			}
//			BATT_Alarm_LED(); //电池低电压报警	  
		}
		if(IRQ_Scan) //5Hz
		{
			IRQ_Scan = 0;

      
			SI24R1_SingalCheck(); //2.4G通信检测
			SendToRemote(); //发送数据给遥控器
		}
		if(Batt_Scan) //2.5Hz
		{
			Batt_Scan = 0;
//			SI24R1_GetAddr(); //分配2.4G地址
			LowVoltage_Alarm();	//低电量报警
		}
}



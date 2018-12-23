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
#include "remotedata.h"
#include "usart.h"
#include "structconfig.h"
#include "led.h"
#include "power.h"

#define REMOTE_VERSION 0

uint8_t DataID;	//数据包ID
RC_TYPE RC_Control;

void Button_command(uint8_t Button);

/***********************************************************************************************************************
* 函  数：void Remote_Data_ReceiveAnalysis(void)
* 功  能：遥控器数据包解析
* 参  数：无
* 返回值：无
* 备  注：通信协议：
         前导码-按键MASK--ADC1低8--ADC1高8--ADC2低8--ADC2高8--ADC3低8--ADC3高8--ADC4低8--ADC4高8--数据包标识--校验码0xa5;
         前导码只有0x01和0x08才表示有效的数据包，0x01表示此数据包是由ADC采样完成触发的，0x08表示此数据包是由遥控器上的按
         键触发的;
		 数据包标识用于识别是否是同一数据包的作用（这在飞机上主要用于当遥控信号中断时，自动开始降落。）
************************************************************************************************************************/
void Remote_Data_ReceiveAnalysis(void)
{
  static uint8_t Pre_mode = 0;
	SI24R1_Controlflag = 1;
	if(SI24R1_RX_DATA[11]!=0xa5)	//验证校验码是否为0xa5
		return;
	
	if(SI24R1_RX_DATA[0] & 0x01) //当数据包是由遥控器的ADC采样完成时触发发送时
	{	
    if(SI24R1_RX_DATA[1] == 0 )
      Flight_mode = 1;
//    else if(SI24R1_RX_DATA[1] == 1 )
//      Flight_mode = 2;
    if(SI24R1_RX_DATA[1] == 2 && Pre_mode != 2)
    {
      SENSER_FLAG_SET(GYRO_OFFSET);
    }

    
		RC_Control.THROTTLE_TEMP      = SI24R1_RX_DATA[2]<<8|SI24R1_RX_DATA[3];		//ADC2
		RC_Control.YAW = SI24R1_RX_DATA[4]<<8|SI24R1_RX_DATA[5]; 		//ADC1
		RC_Control.PITCH     = SI24R1_RX_DATA[6]<<8|SI24R1_RX_DATA[7];	  	//ADC4
		RC_Control.ROLL    = SI24R1_RX_DATA[8]<<8|SI24R1_RX_DATA[9];		//ADC3
    
#if REMOTE_VERSION
		if(RC_Control.THROTTLE_TEMP > 1600)
		{
				RC_Control.THROTTLE += (RC_Control.THROTTLE_TEMP - 1500)/50;
				if(RC_Control.THROTTLE >= 2000 )
					RC_Control.THROTTLE = 2000;
		}
		else if(RC_Control.THROTTLE_TEMP < 1400)
		{
			RC_Control.THROTTLE += (RC_Control.THROTTLE_TEMP - 1500)/50;
			if(RC_Control.THROTTLE <= 1000)
					RC_Control.THROTTLE =1000;
		}
#else
    RC_Control.THROTTLE = RC_Control.THROTTLE_TEMP;
#endif
		
    Pre_mode = SI24R1_RX_DATA[1];
	}

	DataID = SI24R1_RX_DATA[10];//将数据包识别PID值取出，覆盖之前的值，以表示信号链接正常
  
  
}
void Deblocking(void)
{
	 static int8_t flag=1;
	 static int16_t time1=0,time2=0;

   if(Airplane_Enable && RC_Control.ROLL >= 1800 && RC_Control.PITCH <= 1200 && RC_Control.THROTTLE <= 1200 &&  RC_Control.YAW <= 1200)		
	 {  
			time1++; 
	 }	
	 else 
		 time1=0;
	 if(time1>10 && Airplane_Enable) 
	 { 
			Airplane_Enable = 0; 
			time1 = 0;

	 }

   if(!Airplane_Enable && RC_Control.YAW >= 1800 && RC_Control.PITCH <=1200 && RC_Control.THROTTLE <= 1200 &&  RC_Control.ROLL <= 1200)		
		{
			time2++; 
			
		}	
	 else 
		 time2=0;
	 if(time2>=10 && !Airplane_Enable)
	 {
		  Airplane_Enable = 1; 
			time2 = 0;

	 }

}


/**************************************************************************************************************
* 函  数：void UnControl_Land(void)
* 功  能：信号中断紧急降落
* 参  数：无
* 返回值：无
* 备  注：粗略处理，有待完善
***************************************************************************************************************/
void UnControl_Land(void)       
{
	RC_Control.THROTTLE -= 100;
	if(RC_Control.THROTTLE <= 10)
		RC_Control.THROTTLE = 0;
}

/**************************************************************************************************************
* 函  数：void SI24R1_SingalCheck(void)
* 功  能：信号中断检测
* 参  数：无
* 返回值：无
* 备  注：如果飞机处于解锁状态但是,当前数据包的ID等于前一个数据包的ID，这就说明遥控器与飞机断开连接
***************************************************************************************************************/
void SI24R1_SingalCheck(void)
{
	static uint8_t PreDataID = 250; 
	
	if(SI24R1_Controlflag)
	{
		if(Airplane_Enable && DataID == PreDataID)//飞机与遥控断开连接
		{
//			UnControl_Land(); //紧急降落处理

		}else if(Airplane_Enable && !BATT_LEDflag)//飞机遥控连接正常
		{

		}
		PreDataID = DataID;
	}
}

/**************************************************************************************************************
* 函  数：void SendToRemote(void)
* 功  能：飞机状态数据发送给遥控器
* 参  数：无
* 返回值：无
* 备  注：注意：SI24R1单次发送最大32个字节，请勿越界
***************************************************************************************************************/
void SendToRemote(void)
{
	int16_t temp; 	
	if(Airplane_Enable)
	{
		SENSER_FLAG_SET(FLY_ENABLE); //解锁模式置位
	}
	else
	{
		SENSER_FLAG_RESET(FLY_ENABLE); //上锁模式复位
	}
	SI24R1_TX_DATA[0] = 0xAF;//帧头
	
	SI24R1_TX_DATA[1] = SENSER_OFFSET_FLAG; //标志位组
	
	temp = (int)(BAT.BattAdc); //飞机电池电压
	SI24R1_TX_DATA[2] = Byte1(temp);
	SI24R1_TX_DATA[3] = Byte0(temp);
	temp = (int)(Att_Angle.yaw*10); //航向
	SI24R1_TX_DATA[4] = Byte1(temp);
	SI24R1_TX_DATA[5] = Byte0(temp);
	temp = (int)(Att_Angle.pit*10); //俯仰
	SI24R1_TX_DATA[6] = Byte1(temp);
	SI24R1_TX_DATA[7] = Byte0(temp);
	temp = (int)(Att_Angle.rol*10); //横滚
	SI24R1_TX_DATA[8] = Byte1(temp);
	SI24R1_TX_DATA[9] = Byte0(temp);
	temp = (int)(Bmp280.Altitude*100); //高度留待
	SI24R1_TX_DATA[10] = Byte1(temp);
	SI24R1_TX_DATA[11] = Byte0(temp);
  temp = (int)(Flight_mode);
	SI24R1_TX_DATA[12] = Byte0(temp);   //飞行模式
  temp= Airplane_Enable;
  SI24R1_TX_DATA[13] = Byte0(temp);   //解锁
  
	SI24R1_TxPacket(SI24R1_TX_DATA); //SI24R1发送函数
}



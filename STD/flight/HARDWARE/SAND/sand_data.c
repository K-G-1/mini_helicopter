#include "usart.h"
#include "sand_data.h"
#include "mpu9250.h"
#include "mpu6050.h"
#include "led.h"
#include "IMU.h"
#include "rc.h"
#include "control.h"
#include "param.h"



#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	
//extern u8 Mag_CALIBRATED;






/*****************************************************************/
void sand_2402_data(s16 a_off_x,s16 a_off_y,s16 a_off_z,s16 g_off_x,s16 g_off_y,s16 g_off_z)
{
	u8 sum=0,i=0;


	u8 Tx_buff[32]={0};
	
	Tx_buff[0]=0xAA;
	Tx_buff[1]=0xAA;
	Tx_buff[2]=0xf1;
	Tx_buff[3]=26;
	

	Tx_buff[4]=BYTE1(a_off_x);
	Tx_buff[5]=BYTE0(a_off_x);	
	Tx_buff[6]=BYTE1(a_off_y);
	Tx_buff[7]=BYTE0(a_off_y);
	Tx_buff[8]=BYTE1(a_off_z);
	Tx_buff[9]=BYTE0(a_off_z);
	
	Tx_buff[10]=BYTE1(g_off_x);
	Tx_buff[11]=BYTE0(g_off_x);
	Tx_buff[12]=BYTE1(g_off_y);
	Tx_buff[13]=BYTE0(g_off_y);	
	Tx_buff[14]=BYTE1(g_off_z);
	Tx_buff[15]=BYTE0(g_off_z);
	
//	Tx_buff[16]=BYTE1(Magn_x);
//	Tx_buff[17]=BYTE0(Magn_x);	
//	Tx_buff[18]=BYTE1(Magn_y);
//	Tx_buff[19]=BYTE0(Magn_y);
//	Tx_buff[20]=BYTE1(Magn_z);
//	Tx_buff[21]=BYTE0(Magn_z);	
	
	
//	Tx_buff[22]=BYTE1(sensor.gyro.sand.x);
//	Tx_buff[23]=BYTE0(sensor.gyro.sand.x);	
//	Tx_buff[24]=BYTE1(sensor.gyro.sand.y);
//	Tx_buff[25]=BYTE0(sensor.gyro.sand.y);
//	Tx_buff[26]=BYTE1(sensor.gyro.sand.z);
//	Tx_buff[27]=BYTE0(sensor.gyro.sand.z);	

//	Tx_buff[28]=BYTE1(sensor.acc.quiet.x);
//	Tx_buff[29]=BYTE0(sensor.acc.quiet.x);
	
	for(i=0;i<31;i++)
	{
		sum+=Tx_buff[i];
	}
	
	
	Tx_buff[31]=sum;
	for(i=0;i<31;i++)
	{
		USART_SendData(USART1,Tx_buff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
		

}

void sand_ACC_GYRO_data(void)
{
	int16_t sum=0,i=0;
	int16_t temp;

	u8 Tx_buff[32]={0};
	
	Tx_buff[0]=0xAA;
	Tx_buff[1]=0xAA;
	Tx_buff[2]=02;
	Tx_buff[3]=18;
	

	Tx_buff[4]=BYTE1(sensor.acc.averag.x);
	Tx_buff[5]=BYTE0(sensor.acc.averag.x);	
	Tx_buff[6]=BYTE1(sensor.acc.averag.y);
	Tx_buff[7]=BYTE0(sensor.acc.averag.y);
	Tx_buff[8]=BYTE1(sensor.acc.averag.z);
	Tx_buff[9]=BYTE0(sensor.acc.averag.z);
	
	temp = sensor.gyro.radian.x *RtA;
	Tx_buff[10]=BYTE1(temp);
	Tx_buff[11]=BYTE0(temp);
	temp = sensor.gyro.radian.y *RtA;
	Tx_buff[12]=BYTE1(temp);
	Tx_buff[13]=BYTE0(temp);	
	temp = sensor.gyro.radian.z *RtA;
	Tx_buff[14]=BYTE1(temp);
	Tx_buff[15]=BYTE0(temp);
	
	Tx_buff[16]=BYTE1(sensor.mag.origin.x);
	Tx_buff[17]=BYTE0(sensor.mag.origin.x);	
	Tx_buff[18]=BYTE1(sensor.mag.origin.y);
	Tx_buff[19]=BYTE0(sensor.mag.origin.y);
	Tx_buff[20]=BYTE1(sensor.mag.origin.z);
	Tx_buff[21]=BYTE0(sensor.mag.origin.z);	
	
	
//	Tx_buff[22]=BYTE1(sensor.gyro.sand.x);
//	Tx_buff[23]=BYTE0(sensor.gyro.sand.x);	
//	Tx_buff[24]=BYTE1(sensor.gyro.sand.y);
//	Tx_buff[25]=BYTE0(sensor.gyro.sand.y);
//	Tx_buff[26]=BYTE1(sensor.gyro.sand.z);
//	Tx_buff[27]=BYTE0(sensor.gyro.sand.z);	

//	Tx_buff[28]=BYTE1(sensor.acc.quiet.x);
//	Tx_buff[29]=BYTE0(sensor.acc.quiet.x);
	
	for(i=0;i<22;i++)
	{
		sum+=Tx_buff[i];
	}
	
	
	Tx_buff[22]=sum;
	for(i=0;i<23;i++)
	{
		USART_SendData(USART1,Tx_buff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
		

}
extern int att_cnt ;
void sand_IMU_data(void)
{
	u8 sum=0,i=0;
	int16_t temp;
	u8 Tx_buff[32]={0};
	
	Tx_buff[0]=0xAA;
	Tx_buff[1]=0xAA;
	Tx_buff[2]=01;
	Tx_buff[3]=12;
	
	temp=(angle.roll*100);
	Tx_buff[4]=BYTE1(temp);
	Tx_buff[5]=BYTE0(temp);
	
	temp=(angle.pitch*100);
	Tx_buff[6]=BYTE1(temp);
	Tx_buff[7]=BYTE0(temp);	
	
	temp=(angle.yaw*100);
	Tx_buff[8]=BYTE1(temp);
	Tx_buff[9]=BYTE0(temp);
	
    temp = att_cnt*2000;
//	Tx_buff[10]=BYTE3(temp);
//	Tx_buff[11]=BYTE2(temp);	
	Tx_buff[12]=BYTE1(temp);
	Tx_buff[13]=BYTE0(temp);
    //飞行模式
    temp = 1;
	Tx_buff[14]=BYTE0(temp);
    //解锁
    temp = ARMED;
	Tx_buff[15]=BYTE0(temp);
	
	for(i=0;i<16;i++)
	{
		sum+=Tx_buff[i];
	}
	
	
	Tx_buff[16]=sum;
	for(i=0;i<17;i++)
	{
		USART_SendData(USART1,Tx_buff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
}


void sand_RC_data(void)
{
	u8 sum=0,i=0;
    uint16_t temp = 0;

	u8 Tx_buff[32]={0};
	
	Tx_buff[0]=0xAA;
	Tx_buff[1]=0xAA;
	Tx_buff[2]=03;
	Tx_buff[3]=20;
	
    
	Tx_buff[4]=BYTE1(Rc_Data.THROTTLE);
	Tx_buff[5]=BYTE0(Rc_Data.THROTTLE);	
	Tx_buff[6]=BYTE1(Rc_Data.YAW);
	Tx_buff[7]=BYTE0(Rc_Data.YAW);
	Tx_buff[8]=BYTE1(Rc_Data.ROLL);
	Tx_buff[9]=BYTE0(Rc_Data.ROLL);
	Tx_buff[10]=BYTE1(Rc_Data.PITCH);
	Tx_buff[11]=BYTE0(Rc_Data.PITCH);

    temp = 1000;
	Tx_buff[12]=BYTE1(temp);
	Tx_buff[13]=BYTE0(temp);
	
	Tx_buff[14]=BYTE1(temp);
	Tx_buff[15]=BYTE0(temp);
	
	Tx_buff[16]=BYTE1(temp);
	Tx_buff[17]=BYTE0(temp);	
    
	Tx_buff[18]=BYTE1(temp);
	Tx_buff[19]=BYTE0(temp);
    
    Tx_buff[20]=BYTE1(temp);
	Tx_buff[21]=BYTE0(temp);
    
    Tx_buff[22]=BYTE1(temp);
	Tx_buff[23]=BYTE0(temp);
	

	
	for(i=0;i<24;i++)
	{
		sum+=Tx_buff[i];
	}
	
	
	Tx_buff[24]=sum;
	for(i=0;i<25;i++)
	{
		USART_SendData(USART1,Tx_buff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
}


extern u16 Moto_duty[4];
void sand_Motor_data(void)
{
	u8 sum=0,i=0;
    uint16_t temp = 0;

	u8 Tx_buff[32]={0};
	
	Tx_buff[0]=0xAA;
	Tx_buff[1]=0xAA;
	Tx_buff[2]=06;
	Tx_buff[3]=16;
	

	Tx_buff[4]=BYTE1(Moto_duty[0]);
	Tx_buff[5]=BYTE0(Moto_duty[0]);	
	Tx_buff[6]=BYTE1(Moto_duty[1]);
	Tx_buff[7]=BYTE0(Moto_duty[1]);
	Tx_buff[8]=BYTE1(Moto_duty[2]);
	Tx_buff[9]=BYTE0(Moto_duty[2]);
	Tx_buff[10]=BYTE1(Moto_duty[3]);
	Tx_buff[11]=BYTE0(Moto_duty[3]);
    
	Tx_buff[12]=BYTE1(temp);
	Tx_buff[13]=BYTE0(temp);	
	Tx_buff[14]=BYTE1(temp);
	Tx_buff[15]=BYTE0(temp);
	
	Tx_buff[16]=BYTE1(temp);
	Tx_buff[17]=BYTE0(temp);	
	Tx_buff[18]=BYTE1(temp);
	Tx_buff[19]=BYTE0(temp);
	
	for(i=0;i<20;i++)
	{
		sum+=Tx_buff[i];
	}
	
	
	Tx_buff[20]=sum;
	for(i=0;i<21;i++)
	{
		USART_SendData(USART1,Tx_buff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
}




void sand_PID_shell_data(void)
{
	u8 sum=0,i=0;
	vs16 _temp;

	u8 Tx_buff[32]={0};
	
	Tx_buff[0]=0xAA;
	Tx_buff[1]=0xAA;
	Tx_buff[2]=0x11;
	Tx_buff[3]=18;
	
	_temp= ctrl.roll.shell.kp *1000 ;
	Tx_buff[4]=BYTE1(_temp);
	Tx_buff[5]=BYTE0(_temp);
	
	_temp= ctrl.roll.shell.ki *1000 ;
	Tx_buff[6]=BYTE1(_temp);
	Tx_buff[7]=BYTE0(_temp);
	
	_temp= ctrl.roll.shell.kd *1000;
	Tx_buff[8]=BYTE1(_temp);
	Tx_buff[9]=BYTE0(_temp);
	
	
	_temp= 1000 * ctrl.pitch.shell.kp;
	Tx_buff[10]=BYTE1(_temp);
	Tx_buff[11]=BYTE0(_temp);
	
	_temp= 1000 * ctrl.pitch.shell.ki;	
	Tx_buff[12]=BYTE1(_temp);
	Tx_buff[13]=BYTE0(_temp);

	_temp= 1000 * ctrl.pitch.shell.kd;
	Tx_buff[14]=BYTE1(_temp);
	Tx_buff[15]=BYTE0(_temp);
	
	
	_temp= 1000 * ctrl.yaw.shell.kp;
	Tx_buff[16]=BYTE1(_temp);
	Tx_buff[17]=BYTE0(_temp);

	_temp= 1000 * ctrl.yaw.shell.ki;
	Tx_buff[18]=BYTE1(_temp);
	Tx_buff[19]=BYTE0(_temp);
	
		_temp= 1000 * ctrl.yaw.shell.kd;
	Tx_buff[20]=BYTE1(_temp);
	Tx_buff[21]=BYTE0(_temp);	
	
	
	
	for(i=0;i<22;i++)
	{
		sum+=Tx_buff[i];
	}
	
	
	Tx_buff[22]=sum;
	for(i=0;i<23;i++)
	{
		USART_SendData(USART1,Tx_buff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
}



void sand_PID_core_data(void)
{
	u8 sum=0,i=0;
  vs16 _temp;

	u8 Tx_buff[32]={0};
	
	Tx_buff[0]=0xAA;
	Tx_buff[1]=0xAA;
	Tx_buff[2]=0x10;
	Tx_buff[3]=18;
	
	_temp= 1000 * ctrl.roll.core.kp;
	Tx_buff[4]=BYTE1(_temp);
	Tx_buff[5]=BYTE0(_temp);

  _temp= 1000 * ctrl.roll.core.ki;	
	Tx_buff[6]=BYTE1(_temp);
	Tx_buff[7]=BYTE0(_temp);
	
	_temp= 1000 * ctrl.roll.core.kd;
	Tx_buff[8]=BYTE1(_temp);
	Tx_buff[9]=BYTE0(_temp);
	
	
	_temp= 1000 * ctrl.pitch.core.kp;
	Tx_buff[10]=BYTE1(_temp);
	Tx_buff[11]=BYTE0(_temp);
	
	_temp= 1000 * ctrl.roll.core.ki;
	Tx_buff[12]=BYTE1(_temp);
	Tx_buff[13]=BYTE0(_temp);

  _temp= 1000 * ctrl.pitch.core.kd;
	Tx_buff[14]=BYTE1(_temp);
	Tx_buff[15]=BYTE0(_temp);
	
	
	_temp= 1000 * ctrl.yaw.core.kp;
	Tx_buff[16]=BYTE1(_temp);
	Tx_buff[17]=BYTE0(_temp);	
	
	_temp= 1000 * 0;
	Tx_buff[18]=BYTE1(_temp);
	Tx_buff[19]=BYTE0(_temp);
	
	_temp= 1000 * ctrl.yaw.core.kd;
	Tx_buff[20]=BYTE1(_temp);
	Tx_buff[21]=BYTE0(_temp);	
	

	for(i=0;i<22;i++)
	{
		sum+=Tx_buff[i];
	}
	
	
	Tx_buff[22]=sum;
	for(i=0;i<23;i++)
	{
		USART_SendData(USART1,Tx_buff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
}

void sand_PID_3_data(void)
{
	u8 sum=0,i=0;
	int16_t _temp;

	u8 Tx_buff[32]={0};
	
	Tx_buff[0]=0xAA;
	Tx_buff[1]=0xAA;
	Tx_buff[2]=12;
	Tx_buff[3]=18;
	
	for(i=0;i<31;i++)
	{
		sum+=Tx_buff[i];
	}
	
	
	Tx_buff[31]=sum;
	for(i=0;i<31;i++)
	{
		USART_SendData(USART1,Tx_buff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
}
void sand_PID_4_data(void)
{
	u8 sum=0,i=0;
	int16_t _temp;

	u8 Tx_buff[32]={0};
	
	Tx_buff[0]=0xAA;
	Tx_buff[1]=0xAA;
	Tx_buff[2]=13;
	Tx_buff[3]=18;
	
	for(i=0;i<31;i++)
	{
		sum+=Tx_buff[i];
	}
	
	
	Tx_buff[31]=sum;
	for(i=0;i<31;i++)
	{
		USART_SendData(USART1,Tx_buff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
}
void sand_PID_5_data(void)
{
	u8 sum=0,i=0;
	int16_t _temp;

	u8 Tx_buff[32]={0};
	
	Tx_buff[0]=0xAA;
	Tx_buff[1]=0xAA;
	Tx_buff[2]=14;
	Tx_buff[3]=18;
	
	for(i=0;i<31;i++)
	{
		sum+=Tx_buff[i];
	}
	
	
	Tx_buff[31]=sum;
	for(i=0;i<31;i++)
	{
		USART_SendData(USART1,Tx_buff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
}
void sand_PID_6_data(void)
{
	u8 sum=0,i=0;
	int16_t _temp;

	u8 Tx_buff[32]={0};
	
	Tx_buff[0]=0xAA;
	Tx_buff[1]=0xAA;
	Tx_buff[2]=15;
	Tx_buff[3]=18;
	
	for(i=0;i<31;i++)
	{
		sum+=Tx_buff[i];
	}
	
	
	Tx_buff[31]=sum;
	for(i=0;i<31;i++)
	{
		USART_SendData(USART1,Tx_buff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
}
/************************************接收数据并处理******************************************************/
static void Send_Check(u8 head,u8 check_sum)
{
	u8 sum=0,i=0;


	u8 Tx_buff[31]={0};
	
	Tx_buff[0]=0xAA;
	Tx_buff[1]=0xAA;
	Tx_buff[2]=0xEF;
	Tx_buff[3]=2;
	Tx_buff[4]=head;
	Tx_buff[5]=check_sum;
	
	

	for(i=0;i<6;i++)
		sum += Tx_buff[i];
	Tx_buff[6]=sum;

	for(i=0;i<7;i++)
	{
		USART_SendData(USART1,Tx_buff[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
	}
}

void Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		Data_Receive_Anl(RxBuffer,_data_cnt+5);
//		Send_Check();
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////

u16 flash_save_en_cnt = 0;
u16 RX_CH[8];

void Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0,i;
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断 校验位
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		// 判断帧头
	
	
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
		{
			sensor.acc.CALIBRATE = 1;
			#if USE_IMU_DEVICE
			Get_6050_offest();
			#else 
			Get_offest();
			#endif
		}
		else if(*(data_buf+4)==0X02)
			sensor.gyro.CALIBRATE = 1;
		else if(*(data_buf+4)==0X03)
		{
			sensor.acc.CALIBRATE = 1;		
			sensor.gyro.CALIBRATE = 1;
            
		}
		else if(*(data_buf+4)==0X04)
		{
			sensor.mag.CALIBRATE = 1;
		}
		else if((*(data_buf+4)>=0X021)&&(*(data_buf+4)<=0X26))
		{
			//acc_3d_calibrate_f = 1;
		}
		else if(*(data_buf+4)==0X20)
		{
			//acc_3d_step = 0; //??,6?????0
		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)                //读取PID
		{
            sand_PID_core_data();
            Send_Check(*(data_buf+2),sum );
			sand_PID_shell_data();
            Send_Check(*(data_buf+2),sum );
			
//			sand_PID_3_data();
//			sand_PID_4_data();
//			sand_PID_5_data();
//			sand_PID_6_data();
			

		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//    读取版本
		{
			
		}
		if(*(data_buf+4)==0XA1)		//    恢复默认
		{

		}
	}

	
/*
	if(*(data_buf+2)==0X03)
	{
		if( NS != 1 )
		{
			Feed_Rc_Dog(2);
		}

		RX_CH[THR] = (vs16)(*(data_buf+4)<<8)|*(data_buf+5) ;
		RX_CH[YAW] = (vs16)(*(data_buf+6)<<8)|*(data_buf+7) ;
		RX_CH[ROL] = (vs16)(*(data_buf+8)<<8)|*(data_buf+9) ;
		RX_CH[PIT] = (vs16)(*(data_buf+10)<<8)|*(data_buf+11) ;
		RX_CH[AUX1] = (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ;
		RX_CH[AUX2] = (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ;
		RX_CH[AUX3] = (vs16)(*(data_buf+16)<<8)|*(data_buf+17) ;
		RX_CH[AUX4] = (vs16)(*(data_buf+18)<<8)|*(data_buf+19) ;
	}
*/

/************************************上位机发送PID数据回来************************************************/


	if(*(data_buf+2)==0X10)								//PID1
	{
		Send_Check(*(data_buf+2),sum );
		ctrl.roll.core.kp= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		ctrl.roll.core.ki= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		ctrl.roll.core.kd= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		
		ctrl.pitch.core.kp= 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		ctrl.pitch.core.ki= 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		ctrl.pitch.core.kd= 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
		
		ctrl.yaw.core.kp= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
		ctrl.yaw.core.ki= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
		ctrl.yaw.core.kd= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		
        Save_PID_core();
		
	}
	if(*(data_buf+2)==0X11)								//PID2
	{
		Send_Check(*(data_buf+2),sum);
		ctrl.roll.shell.kp= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		ctrl.roll.shell.ki= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		ctrl.roll.shell.kd= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		
		ctrl.pitch.shell.kp= 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		ctrl.pitch.shell.ki= 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		ctrl.pitch.shell.kd= 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
		
		ctrl.yaw.shell.kp= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
		ctrl.yaw.shell.ki= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
		ctrl.yaw.shell.kd= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
			
        Save_PID_shell();
	}
	if(*(data_buf+2)==0X12)								//PID3
	{
		Send_Check(*(data_buf+2),sum);
		
		ctrl.height.shell.kp= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		ctrl.height.shell.ki= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		ctrl.height.shell.kd= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		

	}
	if(*(data_buf+2)==0X13)								//PID4
	{
		Send_Check(*(data_buf+2),sum);
		
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		Send_Check(*(data_buf+2),sum);
		
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		Send_Check(*(data_buf+2),sum);
		
	}
	
	

}





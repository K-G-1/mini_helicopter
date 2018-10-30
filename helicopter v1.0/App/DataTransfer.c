#include "DataTransfer.h"
#include "appconfig.h"
#include "usart.h"
#include "stm32f10x.h"
#include "control.h"
#include "nrf24l01.h"
#include "PWM_output.h"
#include "ADC.h"
#define DATA_TRANSFER_USE_USART

u8 g_u8a50_DataSend_SMS[50]={0};

void g_v_DataSendStatus(void)
{
	u8 _cnt=0;
	vs16 _temp=0;
	u8 sum = 0;
	vs32 _temp2 = g_vs32_Alt_CMS;
	u8 i=0;
	g_u8a50_DataSend_SMS[_cnt++]=0xAA;
	g_u8a50_DataSend_SMS[_cnt++]=0xAA;
	g_u8a50_DataSend_SMS[_cnt++]=0x01;
	g_u8a50_DataSend_SMS[_cnt++]=0;
	
	_temp = (int)(g_t_AttAngle_CMS.rol *100);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE1(_temp);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(_temp);
	_temp = (int)(g_t_AttAngle_CMS.pit *100);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE1(_temp);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(_temp);
	_temp = (int)(g_t_AttAngle_CMS.yaw *100);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE1(_temp);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(_temp);
	
	g_u8a50_DataSend_SMS[_cnt++]=BYTE3(_temp2);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE2(_temp2);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE1(_temp2);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(_temp2);
		
	if(g_t_Rc_C_CMS.ARMED==0)			g_u8a50_DataSend_SMS[_cnt++]=0xA0;	//Ëø¶¨
	else if(g_t_Rc_C_CMS.ARMED==1)		g_u8a50_DataSend_SMS[_cnt++]=0xA1;
	
	g_u8a50_DataSend_SMS[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += g_u8a50_DataSend_SMS[i];
	g_u8a50_DataSend_SMS[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	g_v_Uart1PutBuf(g_u8a50_DataSend_SMS,_cnt);
#else
	g_v_NrfTxPacket(g_u8a50_DataSend_SMS,_cnt);
#endif
}

void g_v_DataSendPwm(void)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	g_u8a50_DataSend_SMS[_cnt++]=0xAA;
	g_u8a50_DataSend_SMS[_cnt++]=0xAA;
	g_u8a50_DataSend_SMS[_cnt++]=0x02;
	g_u8a50_DataSend_SMS[_cnt++]=0;
//	g_u8a50_DataSend_SMS[_cnt++]=BYTE1((temp_rolpwmP));
//	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(temp_rolpwmP);
//	g_u8a50_DataSend_SMS[_cnt++]=BYTE1(temp_rolpwmI);
//	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(temp_rolpwmI);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE1(g_u16_Pwm1MC_CMS);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(g_u16_Pwm1MC_CMS);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE1(g_u16_Pwm2MC_CMS);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(g_u16_Pwm2MC_CMS);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE1(g_u16_Pwm3MC_CMS);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(g_u16_Pwm3MC_CMS);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE1(g_u16_Pwm4MC_CMS);
	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(g_u16_Pwm4MC_CMS);
//	g_u8a50_DataSend_SMS[_cnt++]=0;
//	g_u8a50_DataSend_SMS[_cnt++]=0;
//	g_u8a50_DataSend_SMS[_cnt++]=0;
//	g_u8a50_DataSend_SMS[_cnt++]=0;
//	g_u8a50_DataSend_SMS[_cnt++]=0;
//	g_u8a50_DataSend_SMS[_cnt++]=0;
	
	g_u8a50_DataSend_SMS[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += g_u8a50_DataSend_SMS[i];
	g_u8a50_DataSend_SMS[_cnt++] = sum;
	
#ifdef DATA_TRANSFER_USE_USART
	g_v_Uart1PutBuf(g_u8a50_DataSend_SMS,_cnt);
#else
	g_v_NrfTxPacket(g_u8a50_DataSend_SMS,_cnt);
#endif
}
void g_v_DataSendRC(void)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	g_u8a50_DataSend_SMS[_cnt++]=0xAA;
	g_u8a50_DataSend_SMS[_cnt++]=0xAA;
	g_u8a50_DataSend_SMS[_cnt++]=0x02;
	g_u8a50_DataSend_SMS[_cnt++]=0;
	g_u8a50_DataSend_SMS[_cnt++]=BYTE1(g_t_Rc_D_CMS.THROTTLE); //ÓÍÃÅ
	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(g_t_Rc_D_CMS.THROTTLE); //
	g_u8a50_DataSend_SMS[_cnt++]=BYTE1(g_t_Rc_D_CMS.YAW); 	   //
	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(g_t_Rc_D_CMS.YAW);      //YAW
	g_u8a50_DataSend_SMS[_cnt++]=BYTE1(g_t_Rc_D_CMS.ROLL);     //
	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(g_t_Rc_D_CMS.ROLL);     //rol
	g_u8a50_DataSend_SMS[_cnt++]=BYTE1(g_t_Rc_D_CMS.PITCH);    //
	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(g_t_Rc_D_CMS.PITCH);    //pit
//	g_u8a50_DataSend_SMS[_cnt++]=0;
//	g_u8a50_DataSend_SMS[_cnt++]=0;
//	g_u8a50_DataSend_SMS[_cnt++]=0;
//	g_u8a50_DataSend_SMS[_cnt++]=0;
//	g_u8a50_DataSend_SMS[_cnt++]=0;
//	g_u8a50_DataSend_SMS[_cnt++]=0;
	
	g_u8a50_DataSend_SMS[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += g_u8a50_DataSend_SMS[i];
	g_u8a50_DataSend_SMS[_cnt++] = sum;
	
#ifdef DATA_TRANSFER_USE_USART
	g_v_Uart1PutBuf(g_u8a50_DataSend_SMS,_cnt);
#else
	g_v_NrfTxPacket(g_u8a50_DataSend_SMS,_cnt);
#endif
}
void g_v_DataSendVbat(void)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	g_u8a50_DataSend_SMS[_cnt++]=0xAA;
	g_u8a50_DataSend_SMS[_cnt++]=0xAA;
	g_u8a50_DataSend_SMS[_cnt++]=0x02;
	g_u8a50_DataSend_SMS[_cnt++]=0;
	g_u8a50_DataSend_SMS[_cnt++]=BYTE1((g_u16s8_BatteryVoltage_SMS));
	g_u8a50_DataSend_SMS[_cnt++]=BYTE0(g_u16s8_BatteryVoltage_SMS);
	
	g_u8a50_DataSend_SMS[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += g_u8a50_DataSend_SMS[i];
	g_u8a50_DataSend_SMS[_cnt++] = sum;
	
#ifdef DATA_TRANSFER_USE_USART
	g_v_Uart1PutBuf(g_u8a50_DataSend_SMS,_cnt);
#else
	g_v_NrfTxPacket(g_u8a50_DataSend_SMS,_cnt);
#endif
}

/*******************************************************************************************
										    声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现其他
	
不可估量的BUG，天际智联不负任何责任。请勿商用！

* 程序版本：V1.01
* 程序日期：2018-8-18
* 程序作者：愤怒的小孩
* 版权所有：西安天际智联信息技术有限公司
*******************************************************************************************/
#include "stm32f10x.h"
#include "delay.h"
#include "stdio.h"
#include "led.h"
#include "si24r1.h"
#include "ANO_DT.h"
#include "remotedata.h"
#include "structconfig.h"


uint8_t LED_Scan = 0;
uint8_t IMU_Scan = 0;
uint8_t MPU_Scan = 0;
uint8_t IRQ_Scan = 0;
uint8_t Batt_Scan = 0;
uint8_t ANO_Scan = 0;


/****************************************************************************************************
* 函  数: void USART1_IRQHandler(void)
* 功  能: USART1中断函数，调试使用
* 参  数: 无
* 返回值: 无
* 备  注: 当插线调参时，用USART1对于连续的数据帧的接收 接收中断与空闲中断配合能解决对报问题
****************************************************************************************************/
void USART1_IRQHandler(void)
{
	uint8_t clear = clear; //定义这个变量是针对编译出现“没有用到这个变量”的警告提示
	uint8_t res;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //接收中断
	{ 
		res = USART1->DR;
		ANO_DT_Data_Receive_Prepare(res); //上位机数据接收与解析
	}else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) //空闲中断
	{
		clear = USART1->SR; //读SR寄存器
		clear = USART1->DR; //读DR寄存器（先读SR,再度DR,就是为了清除IDIE中断）
		
	}
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}

/****************************************************************************************************
* 函  数: void TIM4_IRQHandler(void) 
* 功  能: TIM4定时器中断，1ms进一次中断也就是1000Hz
* 参  数: 无
* 返回值: 无
* 备  注: 此函数是整个程序的运行时基，不同的中断时间对应不同频率；
*         对于一些计算对调用时间要求比较严格时可用此方法；
*         扫描频率 = 1000Hz/分频系数；
****************************************************************************************************/
void TIM4_IRQHandler(void)   //TIM4中断服务函数
{
	static uint16_t ms2 = 0,ms5 = 0,ms10 = 0,ms100 = 0,ms200 = 0,ms400 = 0; //分频系数
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)	//判断是否进入TIM更新中断
	{
		ms2++;
		ms5++;
		ms10++;		
		ms100++;
		ms200++;
		ms400++;
		if(ms2 >= 2)//500Hz
		{
			ms2 = 0;
			ANO_Scan = 1;
		}
		if(ms5 >= 5)//200Hz
		{
			ms5 = 0;
			MPU_Scan = 1;
		}
		if(ms10 >= 10)//100Hz
		{
			ms10 = 0;
			IMU_Scan = 1;
		}
		if(ms100 >= 100)//10Hz
		{
			ms100 = 0;
			LED_Scan = 1;
		}
		if(ms200 >= 200)//5Hz
		{
			ms200 = 0;
			IRQ_Scan = 1;
		}
		if(ms400 >= 400)//2.5Hz
		{
			ms400 = 0;
			Batt_Scan = 1;
		}
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	//清除TIM4更新中断
}

/*****************************************************************************
* 函  数：void EXTI2_IRQHandler(void)
* 功  能：SI24R1(全双工)的外中断处理函数
* 参  数：无
* 返回值：无
* 备  注: SI24R1的所有中断事件都在此函数中给予相应的处理
*****************************************************************************/
void EXTI0_IRQHandler(void)
{                    
	uint8_t sta;
	if(EXTI_GetITStatus(EXTI_Line0) != RESET )
	{
//		 SI24R1_CE_LOW;//拉低CE，以便读取NRF中STATUS中的数据
		 sta = SI24R1_read_reg(R_REGISTER+STATUS); //读取STATUS中的数据，以便判断是由什么中断源触发的IRQ中断
		
		/* 发送完成中断 TX_OK */
		if(sta & TX_OK)                                   
		{										
			SI24R1set_Mode(IT_RX);	
			SI24R1_write_reg(W_REGISTER+STATUS,TX_OK); //清除发送完成标志·
			SI24R1_write_reg(FLUSH_TX,0xff); //清除TX_FIFO
//			printf("Sent OK!!!!\r\n");
		}
		/* 接收完成中断 RX_OK */
		if(sta & RX_OK) 
		{	
			SI24R1_RxPacket(SI24R1_RX_DATA);	
			Remote_Data_ReceiveAnalysis();
			SI24R1_write_reg(W_REGISTER+STATUS,RX_OK); //清除发送完成标志·
//			printf("Receive OK!!!!\r\n");	
		}
		/* 达到最大重发次数中断  MAX_TX */
		if(sta & MAX_TX)                                  
		{											
			SI24R1set_Mode(IT_RX);
			SI24R1_write_reg(W_REGISTER+STATUS,MAX_TX);//清除接达到最大重发标志
			SI24R1_write_reg(FLUSH_TX,0xff); //清除TX_FIFO
//			printf("Sent Max Data!!!\r\n"); 
		}
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}



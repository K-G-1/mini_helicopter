/**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    05-December-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"
#include "delay.h"
#include "stdio.h"
#include "led.h"
#include "si24r1.h"
#include "ANO_DT.h"
#include "remotedata.h"
#include "structconfig.h"
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
extern volatile uint32_t sysTickUptime,SYS_Time;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t LED_Scan = 0;
uint8_t IMU_Scan = 0;
uint8_t MPU_Scan = 0;
uint8_t IRQ_Scan = 0;
uint8_t Batt_Scan = 0;
uint8_t ANO_Scan = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
  /* This interrupt is generated when HSE clock fails */

  if (RCC_GetITStatus(RCC_IT_CSS) != RESET)
  {
    /* At this stage: HSE, PLL are disabled (but no change on PLL config) and HSI
       is selected as system clock source */

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Enable HSE Ready and PLL Ready interrupts */
    RCC_ITConfig(RCC_IT_HSERDY | RCC_IT_PLLRDY, ENABLE);

    /* Clear Clock Security System interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_CSS);

    /* Once HSE clock recover, the HSERDY interrupt is generated and in the RCC ISR
       routine the system clock will be reconfigured to its previous state (before
       HSE clock failure) */
  }
}


/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  sysTickUptime--;
}

void USART1_IRQHandler(void)
{
	uint8_t clear = clear; //定义这个变量是针对编译出现“没有用到这个变量”的警告提示
	uint8_t res;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //接收中断
	{ 
//		res = USART1->RDR;
//		ANO_DT_Data_Receive_Prepare(res); //上位机数据接收与解析
    USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) //空闲中断
	{
		USART1->ICR |= (1<<4);										//清除IDLE标志位
		
	}
	
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
void TIM2_IRQHandler(void)   //TIM4中断服务函数
{
	static uint16_t ms2 = 0,ms5 = 0,ms10 = 0,ms100 = 0,ms200 = 0,ms400 = 0; //分频系数
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)	//判断是否进入TIM更新中断
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
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);	//清除TIM4更新中断
}
/*****************************************************************************
* 函  数：void EXTI2_IRQHandler(void)
* 功  能：SI24R1(全双工)的外中断处理函数
* 参  数：无
* 返回值：无
* 备  注: SI24R1的所有中断事件都在此函数中给予相应的处理
*****************************************************************************/
void EXTI0_1_IRQHandler(void)
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

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

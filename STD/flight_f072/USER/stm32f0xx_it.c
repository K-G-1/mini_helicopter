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
	uint8_t clear = clear; //���������������Ա�����֡�û���õ�����������ľ�����ʾ
	uint8_t res;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //�����ж�
	{ 
//		res = USART1->RDR;
//		ANO_DT_Data_Receive_Prepare(res); //��λ�����ݽ��������
    USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) //�����ж�
	{
		USART1->ICR |= (1<<4);										//���IDLE��־λ
		
	}
	
}

/****************************************************************************************************
* ��  ��: void TIM4_IRQHandler(void) 
* ��  ��: TIM4��ʱ���жϣ�1ms��һ���ж�Ҳ����1000Hz
* ��  ��: ��
* ����ֵ: ��
* ��  ע: �˺������������������ʱ������ͬ���ж�ʱ���Ӧ��ͬƵ�ʣ�
*         ����һЩ����Ե���ʱ��Ҫ��Ƚ��ϸ�ʱ���ô˷�����
*         ɨ��Ƶ�� = 1000Hz/��Ƶϵ����
****************************************************************************************************/
void TIM2_IRQHandler(void)   //TIM4�жϷ�����
{
	static uint16_t ms2 = 0,ms5 = 0,ms10 = 0,ms100 = 0,ms200 = 0,ms400 = 0; //��Ƶϵ��
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)	//�ж��Ƿ����TIM�����ж�
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
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);	//���TIM4�����ж�
}
/*****************************************************************************
* ��  ����void EXTI2_IRQHandler(void)
* ��  �ܣ�SI24R1(ȫ˫��)�����жϴ�����
* ��  ������
* ����ֵ����
* ��  ע: SI24R1�������ж��¼����ڴ˺����и�����Ӧ�Ĵ���
*****************************************************************************/
void EXTI0_1_IRQHandler(void)
{                    
	uint8_t sta;
	if(EXTI_GetITStatus(EXTI_Line0) != RESET )
	{
//		 SI24R1_CE_LOW;//����CE���Ա��ȡNRF��STATUS�е�����
		 sta = SI24R1_read_reg(R_REGISTER+STATUS); //��ȡSTATUS�е����ݣ��Ա��ж�����ʲô�ж�Դ������IRQ�ж�
		
		/* ��������ж� TX_OK */
		if(sta & TX_OK)                                   
		{										
			SI24R1set_Mode(IT_RX);	
			SI24R1_write_reg(W_REGISTER+STATUS,TX_OK); //���������ɱ�־��
			SI24R1_write_reg(FLUSH_TX,0xff); //���TX_FIFO
//			printf("Sent OK!!!!\r\n");
		}
		/* ��������ж� RX_OK */
		if(sta & RX_OK) 
		{	
			SI24R1_RxPacket(SI24R1_RX_DATA);	
			Remote_Data_ReceiveAnalysis();
			SI24R1_write_reg(W_REGISTER+STATUS,RX_OK); //���������ɱ�־��
//			printf("Receive OK!!!!\r\n");	
		}
		/* �ﵽ����ط������ж�  MAX_TX */
		if(sta & MAX_TX)                                  
		{											
			SI24R1set_Mode(IT_RX);
			SI24R1_write_reg(W_REGISTER+STATUS,MAX_TX);//����Ӵﵽ����ط���־
			SI24R1_write_reg(FLUSH_TX,0xff); //���TX_FIFO
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

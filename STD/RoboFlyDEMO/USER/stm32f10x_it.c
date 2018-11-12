/*******************************************************************************************
										    �� ��
    ����Ŀ�����������ѧϰʹ�ã�����������ֲ�޸ģ������뱣����������Ϣ����ֲ�����г�������
	
���ɹ�����BUG��������������κ����Ρ��������ã�

* ����汾��V1.01
* �������ڣ�2018-8-18
* �������ߣ���ŭ��С��
* ��Ȩ���У��������������Ϣ�������޹�˾
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
* ��  ��: void USART1_IRQHandler(void)
* ��  ��: USART1�жϺ���������ʹ��
* ��  ��: ��
* ����ֵ: ��
* ��  ע: �����ߵ���ʱ����USART1��������������֡�Ľ��� �����ж�������ж�����ܽ���Ա�����
****************************************************************************************************/
void USART1_IRQHandler(void)
{
	uint8_t clear = clear; //���������������Ա�����֡�û���õ�����������ľ�����ʾ
	uint8_t res;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //�����ж�
	{ 
		res = USART1->DR;
		ANO_DT_Data_Receive_Prepare(res); //��λ�����ݽ��������
	}else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) //�����ж�
	{
		clear = USART1->SR; //��SR�Ĵ���
		clear = USART1->DR; //��DR�Ĵ������ȶ�SR,�ٶ�DR,����Ϊ�����IDIE�жϣ�
		
	}
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
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
void TIM4_IRQHandler(void)   //TIM4�жϷ�����
{
	static uint16_t ms2 = 0,ms5 = 0,ms10 = 0,ms100 = 0,ms200 = 0,ms400 = 0; //��Ƶϵ��
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)	//�ж��Ƿ����TIM�����ж�
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
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	//���TIM4�����ж�
}

/*****************************************************************************
* ��  ����void EXTI2_IRQHandler(void)
* ��  �ܣ�SI24R1(ȫ˫��)�����жϴ�����
* ��  ������
* ����ֵ����
* ��  ע: SI24R1�������ж��¼����ڴ˺����и�����Ӧ�Ĵ���
*****************************************************************************/
void EXTI0_IRQHandler(void)
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



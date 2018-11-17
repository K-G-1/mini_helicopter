/*******************************************************************************************
										    �� ��
    ����Ŀ�����������ѧϰʹ�ã�����������ֲ�޸ģ������뱣����������Ϣ����ֲ�����г�������
	
���ɹ�����BUG��������������κ����Ρ��������ã�

* ����汾��V1.01
* �������ڣ�2018-8-18
* �������ߣ���ŭ��С��
* ��Ȩ���У��������������Ϣ�������޹�˾
*******************************************************************************************/
#include "stm32f0xx.h"
#include "stdlib.h"
#include "delay.h"
//#include "structconfig.h"
#include "led.h"


uint8_t Run_flag=1;//����Ʊ�־

/***************************************************************************
* ��  ����void LED_Init(void)
* ��  �ܣ��û�ָʾ�����ų�ʼ��
* ��  ������
* ����ֵ����
* ��  ע: ��
***************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;   
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);   
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13; 
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;	
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;  
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_13);
}

/***************************************************************************
* ��  ����void LED_Run(void)
* ��  �ܣ�ָʾMCU�Ƿ���
* ��  ������
* ����ֵ����
* ��  ע: ��
***************************************************************************/
void LED_Run(void)
{
	static uint8_t flag = 1;
	if(flag)
	{
		flag = 0;
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
	}
	else
	{
		flag = 1;
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	}
}




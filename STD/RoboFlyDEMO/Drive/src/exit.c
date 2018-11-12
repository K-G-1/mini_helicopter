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
#include "exit.h"
#include "delay.h"
#include "stdio.h"

/****************************************************************************************************
* ��  ��: void Exit_Init(void)
* ��  ��: ������SI24R1��IRQ������IO
* ��  ��: ��
* ����ֵ����
* ��  ע: ��
****************************************************************************************************/
void Exit_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;   //����GPIO�ṹ�����
	EXTI_InitTypeDef EXTI_InitStruct;	//�����ⲿ�жϽṹ�����
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);   //ʹ��GPIOB��ʱ�Ӳ���������ʱ��
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);   //PB2�ж���ӳ��
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0;   //����GPIO��2����
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPU;   //����GPIOΪ��������
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;   //����GPIO����
	GPIO_Init(GPIOA,&GPIO_InitStruct);   //GPIO��ʼ������
	
	
	EXTI_InitStruct.EXTI_Line=EXTI_Line0;   //�ж���2
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;   //�ⲿ�ж�ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Falling;   //�½��ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;   //�ж���ʹ��
	EXTI_Init(&EXTI_InitStruct);   //�ⲿ�жϳ�ʼ������
	
}


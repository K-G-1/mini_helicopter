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


/*****************************************************************************
* ��  ����void TIM_Init(void)
* ��  �ܣ�TIM4��ʼ��Ϊ1ms����һ��
* ��  ������
* ����ֵ����
* ��  ע�������ж�ʱ�� Tout = (ARR-1)*(PSC-1)/CK_INT
*****************************************************************************/
void TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;   //���嶨ʱ���ṹ�����
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);   //ʹ��TIM4��ʱ��
	
	TIM_TimeBaseInitStruct.TIM_Period=48-1;   //�����Զ���װ�ص�����ֵ
	TIM_TimeBaseInitStruct.TIM_Prescaler=1000-1;   //����Ԥ��Ƶֵ
	TIM_TimeBaseInitStruct.TIM_ClockDivision=0;   //����ʱ�ӷָ�
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;   //���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);   //��ʱ����ʼ������
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);   //TIM4�ж�ʹ��
	
	TIM_Cmd(TIM2,ENABLE);   //TIM4ʹ��
}


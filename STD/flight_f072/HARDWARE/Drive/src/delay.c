/*******************************************************************************************
										    �� ��
    ����Ŀ�����������ѧϰʹ�ã�����������ֲ�޸ģ������뱣����������Ϣ����ֲ�����г�������
	
���ɹ�����BUG��������������κ����Ρ��������ã�

* ����汾��V1.01
* �������ڣ�2018-8-18
* �������ߣ���ŭ��С��
* ��Ȩ���У��������������Ϣ�������޹�˾
*******************************************************************************************/
#include "delay.h"
#include "stm32f0xx.h"

static uint8_t   D_us=0;		//΢��ϵ��
static uint16_t  D_ms=0;		//����ϵ��
volatile uint32_t sysTickUptime,SYS_Time;
/****************************************************************************************************
* ��  ��: void Delay_Init(void)
* ��  ��: ��ʱ������ʼ��
* ��  ��: ��
* ����ֵ����
* ��  ע: T(s) = 1/F(Hz) //����ʱ��ת����ʽ
****************************************************************************************************/
void Delay_Init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	
	D_us = SystemCoreClock/8000000;
	D_ms = (uint16_t)D_us * 1000;
	
}

/****************************************************************************************************
* ��  ��: void Delay_us(uint32_t nus)
* ��  ��: ΢�뼶��ʱ
* ��  ��: nus ����΢�����
* ����ֵ����
* ��  ע: LOADΪ24λ�Ĵ�����nus �������ֵΪ 2��24�η�(0xFFFFFF)/ D_us(9) = 1864135 us
****************************************************************************************************/
void Delay_us(uint32_t nus)
{
//	uint32_t temp;
//	SysTick->CTRL = 0x00;			//�ر�SysTick��ʱ��
//	SysTick->LOAD = nus*D_us; 		//��ʱ��װ��ֵ	  		 
//	SysTick->VAL  = 0x00;        	//��ռ�����
//	SysTick->CTRL|= 0x01 ;			//����SysTick��ʱ��  
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));	  	//�ȴ���ʱ����  
//	SysTick->CTRL = 0x00;						//�ر�SysTick��ʱ��
//	SysTick->VAL  = 0X00;      					//��ռ�����
  
  
  	uint32_t temp;	    	 
	SysTick->LOAD=nus*D_us; 					//ʱ�����	  		 
	SysTick->VAL=0x00;        					//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;      					 //��ռ�����	 
}

/****************************************************************************************************
* ��  ��: void Delay_ms(uint32_t nms)
* ��  ��: ���뼶��ʱ
* ��  ��: nms ����΢�����
* ����ֵ����
* ��  ע: LOADΪ24λ�Ĵ�����nms �������ֵΪ 2��24�η�(0xFFFFFF)/ D_ms(9) = 1864 ms
****************************************************************************************************/
void Delay_ms(uint32_t nms)
{
//	uint32_t temp;
//	SysTick->CTRL = 0x00;			//�ر�SysTick��ʱ��
//	SysTick->LOAD = nms*D_ms; 		//��ʱ��װ��ֵ	  		 
//	SysTick->VAL  = 0x00;        	//��ռ�����
//	SysTick->CTRL|= 0x01 ;			//����SysTick��ʱ��  
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));	  	//�ȴ���ʱ����  
//	SysTick->CTRL = 0x00;						//�ر�SysTick��ʱ��
//	SysTick->VAL  = 0X00;      					//��ռ�����	
//  
  
  	uint32_t temp;		   
	SysTick->LOAD=(uint32_t)nms*D_ms;				//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;							//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;       					//��ռ�����	
}


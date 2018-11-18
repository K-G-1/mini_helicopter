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
#include "stdio.h"

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
/*****************************************************************************
* ��  ����void USART_init(uint32_t baudrate)
* ��  �ܣ�Usart1��ʼ��Ϊ˫��ģʽ
* ��  ����baudrate ������
* ����ֵ����
* ��  ע����������������֡�Ľ��� �����ж�������ж�����ܽ���������⣬
          ������շ�ʽ��stm32f1x_it.c �еĴ����жϴ���;
*****************************************************************************/
void USART_init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct;   //����GPIO�ṹ�����
	USART_InitTypeDef USART_InitStruct;   //���崮�ڽṹ�����
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);   //ʹ��GPIOA��USART1��ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
  	//PA.9��PA.10��Ϊ���ù���
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);			
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
  
  /*USART1_TX ->PA9  USART1_RX ->PA10*/		
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;	       //ѡ�д���Ĭ������ܽ�         
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;  //�������������� 
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//����ܽ�9��ģʽ  
  GPIO_Init(GPIOA, &GPIO_InitStruct);           //���ú������ѽṹ�����������г�ʼ��		
	
	USART_InitStruct.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;   //���ͽ���ģʽ
	USART_InitStruct.USART_Parity=USART_Parity_No;   //����żУ��
	USART_InitStruct.USART_BaudRate=baudrate;   //������
	USART_InitStruct.USART_StopBits=USART_StopBits_1;   //ֹͣλ1λ
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;   //�ֳ�8λ
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;   //��Ӳ������������
	USART_Init(USART1,&USART_InitStruct);   //���ڳ�ʼ������
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);		//���ڽ����ж�
//	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);		//���ڿ����ж�
	
	USART_Cmd(USART1,ENABLE);   //ʹ��USART1
}

/*****************************************************************************
* ��  ����int fputc(int ch, FILE *f)
* ��  �ܣ��ض��� printf()����
* ��  ����ch Ҫ���͵�����
* ����ֵ����
* ��  ע����
*****************************************************************************/
//int fputc(int ch,FILE *f)   //printf�ض�����
//{
//	USART_SendData(USART1,(uint8_t)ch);   //����һ�ֽ�����
//	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);   //�ȴ��������
//	return ch;
//}
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until transmit data register is empty */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
  {}

  return ch;
}

/*****************************************************************************
* ��  ����void usart_send(uint8_t *data,uint8_t len)
* ��  �ܣ�Usart����ָ����������
* ��  ����*data Ҫ�������ݵĵ�ַ
*         len   Ҫ�������ݵĳ���
* ����ֵ����
* ��  ע����
*****************************************************************************/
void usart_send(uint8_t *data,uint8_t len)
{
	uint8_t i;
	
	for(i=0;i<len;i++)
	{
		USART_SendData(USART1,*(data+i));
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
	}
}



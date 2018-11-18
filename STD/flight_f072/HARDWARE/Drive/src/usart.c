/*******************************************************************************************
										    声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现其他
	
不可估量的BUG，天际智联不负任何责任。请勿商用！

* 程序版本：V1.01
* 程序日期：2018-8-18
* 程序作者：愤怒的小孩
* 版权所有：西安天际智联信息技术有限公司
*******************************************************************************************/
#include "stm32f0xx.h"
#include "stdio.h"

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
/*****************************************************************************
* 函  数：void USART_init(uint32_t baudrate)
* 功  能：Usart1初始化为双工模式
* 参  数：baudrate 波特率
* 返回值：无
* 备  注：对于连续的数据帧的接收 接收中断与空闲中断配合能解决丢包问题，
          具体接收方式见stm32f1x_it.c 中的串口中断处理;
*****************************************************************************/
void USART_init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct;   //定义GPIO结构体变量
	USART_InitTypeDef USART_InitStruct;   //定义串口结构体变量
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);   //使能GPIOA、USART1的时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
  	//PA.9和PA.10作为复用功能
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);			
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
  
  /*USART1_TX ->PA9  USART1_RX ->PA10*/		
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;	       //选中串口默认输出管脚         
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;  //定义输出最大速率 
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//定义管脚9的模式  
  GPIO_Init(GPIOA, &GPIO_InitStruct);           //调用函数，把结构体参数输入进行初始化		
	
	USART_InitStruct.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;   //发送接收模式
	USART_InitStruct.USART_Parity=USART_Parity_No;   //无奇偶校验
	USART_InitStruct.USART_BaudRate=baudrate;   //波特率
	USART_InitStruct.USART_StopBits=USART_StopBits_1;   //停止位1位
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;   //字长8位
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;   //无硬件数据流控制
	USART_Init(USART1,&USART_InitStruct);   //串口初始化函数
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);		//串口接收中断
//	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);		//串口空闲中断
	
	USART_Cmd(USART1,ENABLE);   //使能USART1
}

/*****************************************************************************
* 函  数：int fputc(int ch, FILE *f)
* 功  能：重定向 printf()函数
* 参  数：ch 要发送的数据
* 返回值：无
* 备  注：无
*****************************************************************************/
//int fputc(int ch,FILE *f)   //printf重定向函数
//{
//	USART_SendData(USART1,(uint8_t)ch);   //发送一字节数据
//	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);   //等待发送完成
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
* 函  数：void usart_send(uint8_t *data,uint8_t len)
* 功  能：Usart发送指定长度数据
* 参  数：*data 要发送数据的地址
*         len   要发送数据的长度
* 返回值：无
* 备  注：无
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



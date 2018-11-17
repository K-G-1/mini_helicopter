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

/*****************************************************************************
* 函  数：void NvicConfig(void)
* 功  能：配置工程中所有中断的优先级
* 参  数：无
* 返回值：无
* 备  注：此优先级中断不要随便更改哦
*****************************************************************************/
void NvicConfig(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel=TIM2_IRQn;   //TIM4中断通道
	NVIC_InitStruct.NVIC_IRQChannelPriority=1;   //抢占优先级0
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;   //使能TIM4中断通道
	NVIC_Init(&NVIC_InitStruct);   //中断优先级初始化函数
	
	NVIC_InitStruct.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPriority=2;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI0_1_IRQn;   //配置外部中断通道
	NVIC_InitStruct.NVIC_IRQChannelPriority=2;   //设置子优先级为1
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;   //使能外部中断通道
	NVIC_Init(&NVIC_InitStruct);   //中断优先级初始化函数
}


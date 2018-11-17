/*******************************************************************************************
										    声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现其他
	
不可估量的BUG，天际智联不负任何责任。请勿商用！

* 程序版本：V1.01
* 程序日期：2018-8-18
* 程序作者：愤怒的小孩
* 版权所有：西安天际智联信息技术有限公司
*******************************************************************************************/
#include "stm32f10x.h"
#include "exit.h"
#include "delay.h"
#include "stdio.h"

/****************************************************************************************************
* 函  数: void Exit_Init(void)
* 功  能: 配置与SI24R1的IRQ相连的IO
* 参  数: 无
* 返回值：无
* 备  注: 无
****************************************************************************************************/
void Exit_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;   //定义GPIO结构体变量
	EXTI_InitTypeDef EXTI_InitStruct;	//定义外部中断结构体变量
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);   //使能GPIOB的时钟并开启复用时钟
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);   //PB2中断线映射
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0;   //配置GPIO第2引脚
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPU;   //配置GPIO为上拉输入
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;   //配置GPIO速率
	GPIO_Init(GPIOA,&GPIO_InitStruct);   //GPIO初始化函数
	
	
	EXTI_InitStruct.EXTI_Line=EXTI_Line0;   //中断线2
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;   //外部中断模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Falling;   //下降沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;   //中断线使能
	EXTI_Init(&EXTI_InitStruct);   //外部中断初始化函数
	
}


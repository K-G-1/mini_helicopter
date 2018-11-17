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
#include "stdlib.h"
#include "delay.h"
//#include "structconfig.h"
#include "led.h"


uint8_t Run_flag=1;//跑马灯标志

/***************************************************************************
* 函  数：void LED_Init(void)
* 功  能：用户指示灯引脚初始化
* 参  数：无
* 返回值：无
* 备  注: 无
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
* 函  数：void LED_Run(void)
* 功  能：指示MCU是否工作
* 参  数：无
* 返回值：无
* 备  注: 无
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




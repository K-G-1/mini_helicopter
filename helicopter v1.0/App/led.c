/********************************************************************************
 * 文件名  ：led.h
 * 描述    ：用3.5.0版本建的工程模板。         
 * 实验平台：野火STM32开发板
 * 硬件连接：-----------------  
 *          |   PC3 - LED1     |        
 *          |   PC4 - LED2     |  
 *          |   PC5 - LED3     |  
 *           -----------------  
 * 库版本  ：ST3.5.0
 *
 * 作者    ：王祥 
 * 论坛    ：
 * 淘宝    ：http://shop105490468.taobao.com/index.htm?spm=a1z10.30.w5002-2929426307.2.QZBskN
**********************************************************************************/

#include "led.h"

/*
 * 函数名：LED_GPIO_Config
 * 描述  ：配置LED用到的I/O口
 * 输入  ：无
 * 输出  ：无
 */
void LED_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
	

//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//enable AFIO and GPIO clock before disable Jtag
	/*选择要控制的GPIOC引脚 | GPIO_Pin_4 | GPIO_Pin_5*/															   
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;	

	/*设置引脚模式为通用推挽输出*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*设置引脚速率为50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*调用库函数，初始化GPIOC*/
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	GPIO_SetBits(GPIOB, GPIO_Pin_12| GPIO_Pin_13|GPIO_Pin_14 );
}
























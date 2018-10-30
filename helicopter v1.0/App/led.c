/********************************************************************************
 * �ļ���  ��led.h
 * ����    ����3.5.0�汾���Ĺ���ģ�塣         
 * ʵ��ƽ̨��Ұ��STM32������
 * Ӳ�����ӣ�-----------------  
 *          |   PC3 - LED1     |        
 *          |   PC4 - LED2     |  
 *          |   PC5 - LED3     |  
 *           -----------------  
 * ��汾  ��ST3.5.0
 *
 * ����    ������ 
 * ��̳    ��
 * �Ա�    ��http://shop105490468.taobao.com/index.htm?spm=a1z10.30.w5002-2929426307.2.QZBskN
**********************************************************************************/

#include "led.h"

/*
 * ��������LED_GPIO_Config
 * ����  ������LED�õ���I/O��
 * ����  ����
 * ���  ����
 */
void LED_GPIO_Config(void)
{		
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
	

//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//enable AFIO and GPIO clock before disable Jtag
	/*ѡ��Ҫ���Ƶ�GPIOC���� | GPIO_Pin_4 | GPIO_Pin_5*/															   
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;	

	/*��������ģʽΪͨ���������*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*������������Ϊ50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*���ÿ⺯������ʼ��GPIOC*/
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	GPIO_SetBits(GPIOB, GPIO_Pin_12| GPIO_Pin_13|GPIO_Pin_14 );
}
























#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly_Remotor
 * ������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/   	 

#define KEY_L  		GPIO_Pin_2
#define KEY_S2   	GPIO_Pin_3

#define READ_KEY_L()  	GPIO_ReadInputDataBit(GPIOA,KEY_L)	//��ȡ�󰴼�
#define READ_KEY_J2()  	GPIO_ReadInputDataBit(GPIOA,KEY_S2)	//��ȡҡ��2����

//IO��ʼ��
void keyInit(void);

 //����ɨ�躯��		
void KEY_Scan(void);

#endif




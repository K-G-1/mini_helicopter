#ifndef __LED_H
#define __LED_H	 
#include "stm32f0xx.h"


enum{
  ON=0,
  OFF
};

#define LED0_Pin  GPIO_Pin_12
#define LED1_Pin  GPIO_Pin_13
#define LED2_Pin  GPIO_Pin_14

#define LED0(x) if(x) GPIO_ResetBits(GPIOB,LED0_Pin); else GPIO_SetBits(GPIOB,LED0_Pin);
#define LED1(x) if(x) GPIO_ResetBits(GPIOB,LED1_Pin); else GPIO_SetBits(GPIOB,LED1_Pin);
#define LED2(x) if(x) GPIO_ResetBits(GPIOB,LED2_Pin); else GPIO_SetBits(GPIOB,LED2_Pin);

void LED_Init(void);//≥ı ºªØ

		 				    
#endif

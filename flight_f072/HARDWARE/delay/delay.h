#ifndef __DELAY_H
#define __DELAY_H 			   
#include "stm32f0xx.h"



void  SysTick_Configuration(void);
uint32_t GetSysTime_us(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void delay_init(void);
  
#endif






























#ifndef __timer_H
#define	__timer_H

#include "stm32f10x.h"


#define START_TIME  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);TIM_Cmd(TIM1, ENABLE)
#define STOP_TIME  TIM_Cmd(TIM1, DISABLE);RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , DISABLE)



extern volatile uint8_t g_u8_TimeFlag1ms_SMS; // 1ms 标志
extern volatile uint8_t g_u8_TimeFlag2ms_SMS; // 2ms 标志
extern volatile uint8_t g_u8_TimeFlag3ms_SMS; // 3ms 标志
extern volatile uint8_t g_u8_TimeFlag4ms_SMS; // 4ms 标志
extern volatile uint8_t g_u8_TimeFlag5ms_SMS; // 5ms 标志

void NVIC_Configuration(void);
void TIM2_Configuration(void);
void TIM1_Configuration(void);
void Tim3_Init(u16 period_num);


#endif

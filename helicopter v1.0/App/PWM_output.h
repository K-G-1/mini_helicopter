#ifndef __PWM_output_H
#define	__PWM_output_H

#include "stm32f10x.h"


extern volatile uint16_t g_u16_Pwm1MC_CMS;
extern volatile uint16_t g_u16_Pwm2MC_CMS;
extern volatile uint16_t g_u16_Pwm3MC_CMS;
extern volatile uint16_t g_u16_Pwm4MC_CMS;

void g_v_PwmInit(void);
void g_v_PwmUpdateDuty(void);

#endif









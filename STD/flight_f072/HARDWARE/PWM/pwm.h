#ifndef __pwm_H
#define __pwm_H


#include "stm32f0xx.h"

void Jtag_disable(void);
void pwm_init(uint16_t arr,uint16_t psc);
void Moto_PwmRflash(uint16_t MOTO1_PWM,uint16_t MOTO2_PWM,uint16_t MOTO3_PWM,uint16_t MOTO4_PWM);


#endif

#ifndef __USART3_H
#define	__USART3_H

#include "stm32f10x.h"
#include <stdio.h>

#define USART_GSM   USART3

void USART3_Config(void);
void NVIC_Configuration(void);
int fputc(int ch, FILE *f);
void USART3_printf(USART_TypeDef* USARTx, uint8_t *Data,...);

#endif /* __USART3_H */




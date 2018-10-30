#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
#include <stdio.h>

#define  Bluetooth_Max  9
#define USART_DEBUG USART1
extern char Rec_Bluetooth_Buf[8];
extern uint8_t j;

void USART1_Config(void);
int fputc(int ch, FILE *f);
void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);
void clearBluetooth_Buff(void);
#endif /* __USART1_H */

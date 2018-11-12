#ifndef   _USART_H
#define   _USART_H
#include "stdint.h"


void USART_init(uint32_t baudrate);
void usart_send(uint8_t *data,uint8_t len);

#endif


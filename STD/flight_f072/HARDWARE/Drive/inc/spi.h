#ifndef   _SPI_H
#define   _SPI_H

#include "stm32f0xx.h"
#include "stdint.h"

void SPI_GPIO_Init(void);
uint8_t SPI1_WriteReadByte(uint8_t data);

uint8_t SPI_Write_byte(SPI_TypeDef* SPIx,uint8_t data);
uint8_t SPI_Read_byte(SPI_TypeDef* SPIx,uint8_t data);
#endif


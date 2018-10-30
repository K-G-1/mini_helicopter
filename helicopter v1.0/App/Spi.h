#ifndef _SPI_H_
#define _SPI_H_
#include "stm32f10x.h"

	
#define NRF_Read_IRQ()		  GPIO_ReadInputDataBit ( GPIOA, GPIO_Pin_12)  //ÖÐ¶ÏÒý½Å



#define SPI_CE_H()   GPIO_SetBits(GPIOA,GPIO_Pin_8)
#define SPI_CE_L()   GPIO_ResetBits(GPIOA,GPIO_Pin_8)		

#define SPI_CSN_H()  GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define SPI_CSN_L()  GPIO_ResetBits(GPIOA, GPIO_Pin_11)	

void Spi2_Init(void);
u8 Spi_RW(u8 dat);

#endif

#ifndef _I2C_H
#define _I2C_H
#include "stm32f10x.h"

//如果移植程序时只要改一下三个地方就行了
#define I2C_SCL GPIO_Pin_10
#define I2C_SDA GPIO_Pin_11
#define GPIO_I2C GPIOB

#define I2C_SCL_H GPIO_SetBits(GPIO_I2C,I2C_SCL)
#define I2C_SCL_L GPIO_ResetBits(GPIO_I2C,I2C_SCL)

#define I2C_SDA_H GPIO_SetBits(GPIO_I2C,I2C_SDA)
#define I2C_SDA_L GPIO_ResetBits(GPIO_I2C,I2C_SDA)

void I2C_Init1(void);
void I2C_SDA_OUT(void);
void I2C_SDA_IN(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NAck(void);
uint8_t   I2C_Wait_Ack(void);
void I2C_Send_Byte(u8 txd);
uint8_t   I2C_Read_Byte(u8 ack);

uint8_t I2CwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data);
uint8_t I2CwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
unsigned char I2CwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
uint8_t I2CreadByte(u8 dev, u8 reg, u8 *data);
uint8_t I2CwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
uint8_t I2CreadBytes(u8 dev, u8 reg, u8 length, u8 *data);
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);


#endif

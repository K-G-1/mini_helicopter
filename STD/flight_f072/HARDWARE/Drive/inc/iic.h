#ifndef   _IIC_H
#define   _IIC_H
#include "stdint.h"

#define SCL     GPIO_Pin_6 //24C02 SCL
#define SDA     GPIO_Pin_7 //24C02 SDA 

#define SDA_H               GPIO_SetBits(GPIOB,SDA)          /*SDA做输出*/
#define SDA_L               GPIO_ResetBits(GPIOB,SDA)        /*SDA做输入*/
#define SCL_H               GPIO_SetBits(GPIOB,SCL)          /*SCL做输出*/
#define SCL_L               GPIO_ResetBits(GPIOB,SCL)        /*SCL做输入*/
#define SDA_READ            GPIO_ReadInputDataBit(GPIOB,SDA) /* 读取SDA*/

void IIC_GPIO_Init(void);        //初始化IIC的IO口				 
void IIC_Start(void);			 //发送IIC开始信号
void IIC_Stop(void);	  	  	 //发送IIC停止信号
void IIC_Ack(void);				 //IIC发送ACK信号
void IIC_NAck(void);			 //IIC不发送ACK信号
uint8_t IIC_WaitAck(void); 		 //IIC等待ACK信号

void IIC_SendByte(uint8_t data);  //IIC发送一个字节
uint8_t IIC_ReadByte(uint8_t ack);//IIC读取一个字节

uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);

void IIC_ADD_write(uint8_t DeviceAddr,uint8_t address,uint8_t Bytes);
uint8_t IIC_ADD_read(uint8_t DeviceAddr,uint8_t address);

#endif


#ifndef _IIC_H_
#define _IIC_H_
#include "stm32f0xx.h"
/*函数声明----------------------------------------------------------------*/

#define SCL     GPIO_Pin_6 //24C02 SCL
#define SDA     GPIO_Pin_7 //24C02 SDA 

//IO方向设置
#define SDA_IN()  {GPIOB->MODER&=0xFFFF3FFF;}
#define SDA_OUT() {GPIOB->MODER&=0X0FFF7FFF;}
//IO操作函数	

#define SDA_H               GPIO_SetBits(GPIOB,SDA)          /*SDA做输出*/
#define SDA_L               GPIO_ResetBits(GPIOB,SDA)        /*SDA做输入*/
#define SCL_H               GPIO_SetBits(GPIOB,SCL)          /*SCL做输出*/
#define SCL_L               GPIO_ResetBits(GPIOB,SCL)        /*SCL做输入*/
#define SDA_READ            GPIO_ReadInputDataBit(GPIOB,SDA) /* 读取SDA*/




//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
uint16_t IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(void);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	 


//额外函数
void IIC_ADD_write(uint8_t DeviceAddr,uint8_t address,uint8_t Bytes);
uint8_t IIC_ADD_read(uint8_t DeviceAddr,uint8_t address);
uint8_t IIC_Read_MultiBytes(uint8_t DeviceAddr,uint8_t address,uint8_t Len,uint8_t *data);
void IIC_NoAddr_WriteByte(unsigned char address,unsigned char Bytes);

uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);


#endif

//------------------End of File----------------------------


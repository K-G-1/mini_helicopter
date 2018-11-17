#ifndef   _IIC_H
#define   _IIC_H
#include "stdint.h"

#define SCL     GPIO_Pin_6 //24C02 SCL
#define SDA     GPIO_Pin_7 //24C02 SDA 

#define SDA_H               GPIO_SetBits(GPIOB,SDA)          /*SDA�����*/
#define SDA_L               GPIO_ResetBits(GPIOB,SDA)        /*SDA������*/
#define SCL_H               GPIO_SetBits(GPIOB,SCL)          /*SCL�����*/
#define SCL_L               GPIO_ResetBits(GPIOB,SCL)        /*SCL������*/
#define SDA_READ            GPIO_ReadInputDataBit(GPIOB,SDA) /* ��ȡSDA*/

void IIC_GPIO_Init(void);        //��ʼ��IIC��IO��				 
void IIC_Start(void);			 //����IIC��ʼ�ź�
void IIC_Stop(void);	  	  	 //����IICֹͣ�ź�
void IIC_Ack(void);				 //IIC����ACK�ź�
void IIC_NAck(void);			 //IIC������ACK�ź�
uint8_t IIC_WaitAck(void); 		 //IIC�ȴ�ACK�ź�

void IIC_SendByte(uint8_t data);  //IIC����һ���ֽ�
uint8_t IIC_ReadByte(uint8_t ack);//IIC��ȡһ���ֽ�

uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);

void IIC_ADD_write(uint8_t DeviceAddr,uint8_t address,uint8_t Bytes);
uint8_t IIC_ADD_read(uint8_t DeviceAddr,uint8_t address);

#endif


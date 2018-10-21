#ifndef _IIC_H_
#define _IIC_H_
#include "stm32f0xx.h"
/*��������----------------------------------------------------------------*/

#define SCL     GPIO_Pin_6 //24C02 SCL
#define SDA     GPIO_Pin_7 //24C02 SDA 

//IO��������
#define SDA_IN()  {GPIOB->MODER&=0xFFFF3FFF;}
#define SDA_OUT() {GPIOB->MODER&=0X0FFF7FFF;}
//IO��������	

#define SDA_H               GPIO_SetBits(GPIOB,SDA)          /*SDA�����*/
#define SDA_L               GPIO_ResetBits(GPIOB,SDA)        /*SDA������*/
#define SCL_H               GPIO_SetBits(GPIOB,SCL)          /*SCL�����*/
#define SCL_L               GPIO_ResetBits(GPIOB,SCL)        /*SCL������*/
#define SDA_READ            GPIO_ReadInputDataBit(GPIOB,SDA) /* ��ȡSDA*/




//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
uint16_t IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t IIC_Read_Byte(void);//IIC��ȡһ���ֽ�
uint8_t IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�
void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	 


//���⺯��
void IIC_ADD_write(uint8_t DeviceAddr,uint8_t address,uint8_t Bytes);
uint8_t IIC_ADD_read(uint8_t DeviceAddr,uint8_t address);
uint8_t IIC_Read_MultiBytes(uint8_t DeviceAddr,uint8_t address,uint8_t Len,uint8_t *data);
void IIC_NoAddr_WriteByte(unsigned char address,unsigned char Bytes);

uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);


#endif

//------------------End of File----------------------------


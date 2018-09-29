#ifndef _IIC_H_
#define _IIC_H_
#include "sys.h"

/*��������----------------------------------------------------------------*/

#define SCL     GPIO_Pin_1 //24C02 SCL
#define SDA     GPIO_Pin_0 //24C02 SDA 

//IO��������
#define SDA_IN()  {GPIOA->CRL&=0XFFFFFFF0;GPIOA->CRL|=(u32)8<<0;}
#define SDA_OUT() {GPIOA->CRL&=0XFFFFFFF0;GPIOA->CRL|=(u32)1<<0;}
//IO��������	
#define IIC_SCL    PAout(1) //SCL
#define IIC_SDA    PAout(0) //SDA
#define SDA_H               GPIO_SetBits(GPIOA,SDA)          /*SDA�����*/
#define SDA_L               GPIO_ResetBits(GPIOA,SDA)        /*SDA������*/
#define SCL_H               GPIO_SetBits(GPIOA,SCL)          /*SCL�����*/
#define SCL_L               GPIO_ResetBits(GPIOA,SCL)        /*SCL������*/
#define SDA_READ            GPIO_ReadInputDataBit(GPIOA,SDA) /* ��ȡSDA*/




//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
u16 IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(void);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�
void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 


//���⺯��
void IIC_ADD_write(u8 DeviceAddr,u8 address,u8 Bytes);
u8 IIC_ADD_read(u8 DeviceAddr,u8 address);
u8 IIC_Read_MultiBytes(u8 DeviceAddr,u8 address,u8 Len,u8 *data);
void IIC_NoAddr_WriteByte(unsigned char address,unsigned char Bytes);

u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);


#endif

//------------------End of File----------------------------


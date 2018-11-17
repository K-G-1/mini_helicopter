/*******************************************************************************************
										    �� ��
    ����Ŀ�����������ѧϰʹ�ã�����������ֲ�޸ģ������뱣����������Ϣ����ֲ�����г�������
	
���ɹ�����BUG��������������κ����Ρ��������ã�

* ����汾��V1.01
* �������ڣ�2018-8-18
* �������ߣ���ŭ��С��
* ��Ȩ���У��������������Ϣ�������޹�˾
*******************************************************************************************/
#include "stm32f10x.h"
#include "si24r1.h"
#include "spi.h"
#include "delay.h"
#include "led.h"
#include "stdio.h"
#include "stdlib.h"
#include "paramsave.h"
#include "remotedata.h"

#define SI24R1AddrMax 50 //NRF���һ���ֽڵ�ַ���Ϊ50

uint8_t SI24R1addr = 0xFF; //��ʼ��NRF���һ�ֽڵ�ַ

uint8_t SI24R1_TX_DATA[TX_PAYLO_WIDTH];//NRF���ͻ�����
uint8_t SI24R1_RX_DATA[RX_PAYLO_WIDTH];//NRF���ջ�����

uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0xAA,0xBB,0xCC,0x00,0x01}; //���͵�ַ
uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0xAA,0xBB,0xCC,0x00,0x01}; //���յ�ַ


/*****************************************************************************
* ��  ����void SI24R1_Init(void)
* ��  �ܣ�NRF����GPIO��ʼ��
* ��  ������
* ����ֵ����
* ��  ע����
*****************************************************************************/
void SI24R1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA,ENABLE);
	
	/*   ����CSN����   */
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	
	/*  ����CE����  */
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		
	SPI_GPIO_Init(); //SPI2��ʼ��

	SI24R1_Check(); //���SI24R1�Ƿ���MCUͨ��                                    

	SI24R1_CSN_HIGH; //ʧ��NRF
	SI24R1_CE_LOW; 	 //����ģʽ
}

/*****************************************************************************
* ��  ����uint8_t SI24R1_write_reg(uint8_t reg,uint8_t value)
* ��  �ܣ�дһ�ֽ����ݵ��Ĵ���
* ��  ����reg�� �Ĵ�����ַ
*         val:  Ҫд�������
* ����ֵ��status
* ��  ע��SI24R1������ֲֻ���SPI�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t SI24R1_write_reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
	
	SI24R1_CSN_LOW;
	status=SPI1_WriteReadByte(reg);
	SPI1_WriteReadByte(value);
	SI24R1_CSN_HIGH;
	
	return status;
}

/*****************************************************************************
* ��  ����uint8_t SI24R1_read_reg(uint8_t reg)
* ��  �ܣ���һ�ֽ����ݵ��Ĵ���
* ��  ����reg�� �Ĵ�����ַ
* ����ֵ��reg_val
* ��  ע��SI24R1������ֲֻ���SPI�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t SI24R1_read_reg(uint8_t reg)
{
	uint8_t reg_val;
	
	SI24R1_CSN_LOW;
	SPI1_WriteReadByte(reg);
	reg_val = SPI1_WriteReadByte(0xff);
	SI24R1_CSN_HIGH;
	
	return reg_val;
}

/*****************************************************************************
* ��  ����uint8_t SI24R1_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
* ��  �ܣ�дһ�����ݵ��Ĵ���
* ��  ����reg�� �Ĵ�����ַ
*         pBuf�� Ҫд�����ݵĵ�ַ
*         len:  Ҫд������ݳ���
* ����ֵ��status
* ��  ע��SI24R1������ֲֻ���SPI�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t SI24R1_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status;
	int i;
	
	SI24R1_CSN_LOW;
	status = SPI1_WriteReadByte(reg);
	for( i=0;i<len;i++)
	{
		SPI1_WriteReadByte(*pBuf);
		pBuf++;
	}
	SI24R1_CSN_HIGH;
	
	return status;
}

/*****************************************************************************
* ��  ����uint8_t SI24R1_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
* ��  �ܣ���һ�����ݵ��Ĵ���
* ��  ����reg�� 	�Ĵ�����ַ
*         pBuf�� Ҫ��ȡ���ݵĵ�ַ
*         len:  	Ҫ��ȡ�����ݳ���
* ����ֵ��status
* ��  ע��SI24R1������ֲֻ���SPI�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t SI24R1_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status;
	int i;
	
	SI24R1_CSN_LOW;
	status = SPI1_WriteReadByte(reg);
	for(i = 0;i < len ;i++)
	{
		*pBuf = SPI1_WriteReadByte(0xff);
		pBuf++;
	}
	SI24R1_CSN_HIGH;
	
	return status;
}

/*****************************************************************************
* ��  ����void SI24R1set_Mode(uint8_t mode)
* ��  �ܣ��л�SI24R1�Ĺ���ģʽģʽ
* ��  ������
* ����ֵ����
* ��  ע����
*****************************************************************************/
void SI24R1set_Mode(uint8_t mode)
{
	if(mode == IT_TX)
	{
		SI24R1_CE_LOW;
		SI24R1_write_reg(W_REGISTER+CONFIG,IT_TX);
		SI24R1_write_reg(W_REGISTER+STATUS,0X7E); //��������ж�,��ֹһ��ȥ����ģʽ�ʹ����ж�	
		SI24R1_CE_HIGH;
//		Delay_us(15);
	}
	else
	{

		SI24R1_write_reg(W_REGISTER+CONFIG,0x0f);//����Ϊ����ģʽ
//		SI24R1_write_reg(W_REGISTER+STATUS,0X7E); //��������ж�,��ֹһ��ȥ����ģʽ�ʹ����ж�

		Delay_us(200);
	}		
}

/*****************************************************************************
* ��  ����void SI24R1_Config(void)
* ��  �ܣ�SI24R1�����������ã�����ʼ��Ϊ����ģʽ
* ��  ������
* ����ֵ����
* ��  ע����
*****************************************************************************/
void SI24R1_Config(void)
{
	SI24R1_CE_LOW;

	SI24R1_Write_Buf(W_REGISTER+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK

	
	SI24R1_write_reg(W_REGISTER+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  
	SI24R1_write_reg(W_REGISTER+EN_AA,0x01); //ʹ��ͨ��0�Զ�Ӧ��
	SI24R1_write_reg(W_REGISTER+RX_PW_P0,RX_PAYLO_WIDTH);//ѡ��ͨ��0����Ч���ݿ��  
	SI24R1_Write_Buf(W_REGISTER+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //дRX�ڵ��ַ
	SI24R1_write_reg(W_REGISTER+RF_CH,40); //����RFͨ��Ϊ40hz(1-64Hz������)
	SI24R1_write_reg(W_REGISTER+RF_SETUP,0x0f); //����TX�������,0db����,2Mbps,����������ر� ��ע�⣺����������ر�/����ֱ��Ӱ��ͨ��,Ҫ������������Ҫ�رն��ر�0x0f��
	
	SI24R1set_Mode(IT_RX); //Ĭ��Ϊ����ģʽ
	SI24R1_CE_HIGH;
}	

/*****************************************************************************
* ��  ����uint8_t SI24R1_TxPacket(uint8_t *txbuf)
* ��  �ܣ�SI24R1����һ������
* ��  ����txbuf��Ҫ�������ݵ�ַ
* ����ֵ���� 
* ��  ע����
*****************************************************************************/
void SI24R1_TxPacket(uint8_t *txbuf)
{
	SI24R1_CE_LOW;	
	SI24R1_Write_Buf(W_REGISTER+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);  //дTX�ڵ��ַ 
	SI24R1_Write_Buf(W_REGISTER+RX_ADDR_P0,(uint8_t*)TX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK
	SI24R1_Write_Buf(W_RX_PAYLOAD,txbuf,TX_PAYLO_WIDTH); //д���ݵ�TX_BUFF
	SI24R1_write_reg(W_REGISTER+CONFIG,0x0e);	//����Ϊ����ģʽ,���������ж�
	SI24R1_write_reg(W_REGISTER+STATUS,0X7E); //��������ж�,��ֹһ��ȥ����ģʽ�ʹ����ж�
	SI24R1_CE_HIGH;
	Delay_us(10);  //CE�����ߵ�ƽ10us
}

/*****************************************************************************
* ��  ����uint8_t SI24R1_RxPacket(uint8_t *rxbuf)
* ��  �ܣ�SI24R1����һ������
* ��  ����rxbuf���������ݴ洢��ַ
* ����ֵ����
* ��  ע����
*****************************************************************************/
void SI24R1_RxPacket(uint8_t *rxbuf)
{

	SI24R1_Read_Buf(R_RX_PAYLOAD,rxbuf,TX_PAYLO_WIDTH);//��ȡRX����Ч����
	SI24R1_write_reg(FLUSH_RX,0xff); //���RX FIFO(ע�⣺��仰�ܱ�Ҫ)

}

/*****************************************************************************
* ��  ����uint8_t SI24R1_testConnection(void)
* ��  �ܣ����SI24R1��MCU��SPI�����Ƿ�ͨ������
* ��  ������
* ����ֵ��1������ 0δ����
* ��  ע����
*****************************************************************************/
uint8_t SI24R1_testConnection(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
  uint8_t check_buf[5];
	uint8_t i; 	 
	SI24R1_Write_Buf(W_REGISTER+TX_ADDR,buf,5); //д��5���ֽڵĵ�ַ.	
	SI24R1_Read_Buf(TX_ADDR,check_buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)
	if(check_buf[i]!=0XA5)break;	 							   
	if(i!=5)return 0; //���24L01����	
	return 1;	//��⵽24L01
}

/*****************************************************************************
* ��  ����void SI24R1_Check(void)
* ��  �ܣ����SI24R1�Ƿ�����
* ��  ������
* ����ֵ����
* ��  ע����
*****************************************************************************/
void SI24R1_Check(void)
{
	while(!SI24R1_testConnection())
	{
		printf("\r SI24R1 no connect...\r\n");
		RGB_LED_Red();//��Ƴ���
	}
}

/*****************************************************************************
* ��  ����void SI24R1_GetAddr(void)
* ��  �ܣ����ɻ���ȡ�ϵ�SI24R1��ȡһ����ַ
* ��  ������
* ����ֵ���� 
* ��  ע���˺�����Ҫ��ң�����Ķ�Ƶ��������ʹ�÷���SI24R1ͨ�Ų��ɹ���
          ����Լ����ĵ�ң������ֱ���ù̶���ַ
*****************************************************************************/
void SI24R1_GetAddr(void)
{
	if(SI24R1addr > SI24R1AddrMax)//�� SI24R1addr����10����˵����ʱSI24R1��δ��ʼ�����
	{
		srand(SysTick->VAL);//�����������
//		printf("SysTick->VAL:%d\r\n",SysTick->VAL);
		SI24R1addr = rand()%SI24R1AddrMax;//�����ȡSI24R1���һλ��ַ����ַ:0~50��
		PID_WriteFlash();//����˵�ַFlash
	}else if(SI24R1addr != TX_ADDRESS[TX_ADR_WIDTH-1])
	{
		TX_ADDRESS[TX_ADR_WIDTH-1] = SI24R1addr;
		RX_ADDRESS[TX_ADR_WIDTH-1] = SI24R1addr;
		SI24R1_Config();
//		printf("SI24R1Addr:%d\r\n",SI24R1addr);
	}
}

/*****************************************************************************
* ��  ����void SI24R1_Test(void)
* ��  �ܣ�SI24R1ͨ�Ų��Ժ���
* ��  ������
* ����ֵ���� 
* ��  ע������ʱ��
*****************************************************************************/
void SI24R1_Test(void)
{
	uint8_t t=0;
	static uint8_t mode,key;
	mode = ' ';
	key=mode;
	for(t=0;t<32;t++)
	{
		key++;
		if(key>('~'))key=' ';
		SI24R1_TX_DATA[t]=key;	
	}
	mode++; 
	if(mode>'~')mode=' ';  	  		
	SI24R1_TxPacket(SI24R1_TX_DATA);
}



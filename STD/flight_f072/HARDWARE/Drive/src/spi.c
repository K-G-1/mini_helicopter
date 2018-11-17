/*******************************************************************************************
										    �� ��
    ����Ŀ�����������ѧϰʹ�ã�����������ֲ�޸ģ������뱣����������Ϣ����ֲ�����г�������
	
���ɹ�����BUG��������������κ����Ρ��������ã�

* ����汾��V1.01
* �������ڣ�2018-8-18
* �������ߣ���ŭ��С��
* ��Ȩ���У��������������Ϣ�������޹�˾
*******************************************************************************************/
#include "stm32f0xx.h"
#include "spi.h"

/*****************************************************************************
* ��  ����void SPI_GPIO_Init(void)
* ��  �ܣ�����SI24R1�� SCK��MISO��MOSI���ţ��Լ�SPI2��ʼ��
* ��  ������
* ����ֵ����
* ��  ע������SPIͨ��ʱһ��Ҫ���������ӻ�ģʽ
*         �����ӻ�ģʽ�� ����״̬ ��ƽ
*		  2.4Gģ��ͨ��ʱ��SPI����һ�㲻����10Mbps
*****************************************************************************/
void SPI_GPIO_Init(void)
{
	SPI_InitTypeDef   SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	
  //
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);
  
	//����SPI��SCK��MISO��MOSI����Ϊ��������ģʽ

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

  SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;	//����Ϊ����ģʽ
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;		//NSS�������
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge;	//��һ��ʱ���ز���
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low;	//����״̬Ϊ�͵�ƽ
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;						//8λ����֡
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_8; 	//SPI������8��Ƶ 	48/8=6M
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;	//ȫ˫��ģʽ
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;					//���ݸ�λ����
	SPI_InitStructure.SPI_CRCPolynomial=7;								//CRC�������ʽ
	SPI_Init(SPI1,&SPI_InitStructure);
	SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
	SPI_Cmd(SPI1,ENABLE);	//SPI2ʹ��

//  SPI1_WriteReadByte(0xff);
}

/*****************************************************************************
* ��  ����uint8_t SPI1_WriteReadByte(uint8_t data)
* ��  �ܣ�SPI2��дһ���ֽ�
* ��  ������
* ����ֵ����
* ��  ע����
*****************************************************************************/
uint8_t SPI1_WriteReadByte(uint8_t data)
{
//	 while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
//	 SPI_I2S_SendData16(SPI1, data);
//	
//	 while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
//	 return SPI_I2S_ReceiveData16(SPI1);
  
  
  uint8_t retry=0;				 	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
  {
    retry++;
    if(retry>200)return 0;
  }			  
  SPI_SendData8(SPI1, data); //ͨ������SPIx����һ������
  retry=0;

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
  {
    retry++;
    if(retry>200)return 0;
  }	  						    
  return SPI_ReceiveData8(SPI1); //����ͨ��SPIx������յ�����			
}

uint8_t SPI_Write_byte(SPI_TypeDef* SPIx,uint8_t data)
{
  while(!SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE));
	 SPI_I2S_SendData16(SPIx, data);
  return 0;
}

uint8_t SPI_Read_byte(SPI_TypeDef* SPIx,uint8_t data)
{
  while(!SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE));
	 return SPI_I2S_ReceiveData16(SPIx);
}

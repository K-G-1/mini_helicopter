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
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	
	//����SPI��SCK��MISO��MOSI����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;	//����Ϊ����ģʽ
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;		//NSS�������
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge;	//��һ��ʱ���ز���
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low;	//����״̬Ϊ�͵�ƽ
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;						//8λ����֡
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_8; 	//SPI������8��Ƶ 	36/8=4.5M
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;	//ȫ˫��ģʽ
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;					//���ݸ�λ����
	SPI_InitStructure.SPI_CRCPolynomial=7;								//CRC�������ʽ
	SPI_Init(SPI1,&SPI_InitStructure);
	
	SPI_Cmd(SPI1,ENABLE);	//SPI2ʹ��

  SPI1_WriteReadByte(0xff);
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
//	 SPI_I2S_SendData(SPI1, data);
//	
//	 while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
//	 return SPI_I2S_ReceiveData(SPI1);
  
  
  uint8_t retry=0;				 	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
  {
    retry++;
    if(retry>200)return 0;
  }			  
  SPI_I2S_SendData(SPI1, data); //ͨ������SPIx����һ������
  retry=0;

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
  {
    retry++;
    if(retry>200)return 0;
  }	  						    
  return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����			
}



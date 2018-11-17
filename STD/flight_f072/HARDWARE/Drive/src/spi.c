/*******************************************************************************************
										    声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现其他
	
不可估量的BUG，天际智联不负任何责任。请勿商用！

* 程序版本：V1.01
* 程序日期：2018-8-18
* 程序作者：愤怒的小孩
* 版权所有：西安天际智联信息技术有限公司
*******************************************************************************************/
#include "stm32f0xx.h"
#include "spi.h"

/*****************************************************************************
* 函  数：void SPI_GPIO_Init(void)
* 功  能：配置SI24R1的 SCK、MISO、MOSI引脚，以及SPI2初始化
* 参  数：无
* 返回值：无
* 备  注：调试SPI通信时一定要分清主机从机模式
*         主机从机模式的 空闲状态 电平
*		  2.4G模块通信时，SPI速率一般不大于10Mbps
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
  
	//配置SPI的SCK，MISO和MOSI引脚为复用推挽模式

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
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;	//配置为主机模式
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;		//NSS软件管理
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge;	//第一个时钟沿捕获
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low;	//空闲状态为低电平
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;						//8位数据帧
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_8; 	//SPI波特率8分频 	48/8=6M
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;	//全双工模式
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;					//数据高位先行
	SPI_InitStructure.SPI_CRCPolynomial=7;								//CRC计算多项式
	SPI_Init(SPI1,&SPI_InitStructure);
	SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
	SPI_Cmd(SPI1,ENABLE);	//SPI2使能

//  SPI1_WriteReadByte(0xff);
}

/*****************************************************************************
* 函  数：uint8_t SPI1_WriteReadByte(uint8_t data)
* 功  能：SPI2读写一个字节
* 参  数：无
* 返回值：无
* 备  注：无
*****************************************************************************/
uint8_t SPI1_WriteReadByte(uint8_t data)
{
//	 while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
//	 SPI_I2S_SendData16(SPI1, data);
//	
//	 while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
//	 return SPI_I2S_ReceiveData16(SPI1);
  
  
  uint8_t retry=0;				 	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
  {
    retry++;
    if(retry>200)return 0;
  }			  
  SPI_SendData8(SPI1, data); //通过外设SPIx发送一个数据
  retry=0;

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)//检查指定的SPI标志位设置与否:接受缓存非空标志位
  {
    retry++;
    if(retry>200)return 0;
  }	  						    
  return SPI_ReceiveData8(SPI1); //返回通过SPIx最近接收的数据			
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

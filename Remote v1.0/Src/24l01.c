
#include "24l01.h"
#include "cmsis_os.h"

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{		
//  uint8_t retry=0;				 	
//  retry = HAL_SPI_Transmit(&hspi1,&TxData,1,0x10);
//  HAL_SPI_Receive(&hspi1,&retry,1,0x10);
//  return retry; //返回通过SPIx最近接收的数据	
	
	static uint8_t txdata,rxdata;
	
	txdata=TxData;
	
	if(HAL_SPI_TransmitReceive(&hspi1,&txdata,&rxdata,1,0xff)!= HAL_OK)
	{
    return 0xff;	
	}
	
	return rxdata;	

}

    
const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0xAA,0xBB,0xCC,0x00,0x01}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0xAA,0xBB,0xCC,0x00,0x01};

//初始化24L01的IO口
void NRF24L01_Init(void)
{ 	
//	GPIO_InitTypeDef GPIO_InitStructure;
//  SPI_InitTypeDef  SPI_InitStructure;

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);	 //使能PB,G端口时钟
//    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_SPI1,  ENABLE );//SPI2时钟使能 		
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				 //PB12上拉 防止W25X的干扰
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);	//初始化指定IO
// 	GPIO_SetBits(GPIOA,GPIO_Pin_4);//上拉				
// 	

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	//PG8 7 推挽 	  
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化指定IO
//  
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;   
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PB1 输入  
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1);//PG6,7,8上拉					 
//		 
//  SPI2_Init();    		//初始化SPI	 
// 
//	SPI_Cmd(SPI1, DISABLE); // SPI外设不使能

//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI设置为双线双向全双工
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//SPI主机
//  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//发送接收8位帧结构
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//时钟悬空低
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//数据捕获于第1个时钟沿
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由软件控制
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//定义波特率预分频的值:波特率预分频值为16
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//数据传输从MSB位开始
//	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
//	SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
// 
//	SPI_Cmd(SPI1, ENABLE); //使能SPI外设
//		
  MX_SPI1_Init();
	NRF24L01_CE_L; 			//使能24L01
	NRF24L01_CSN_H;			//SPI片选取消  
    
	 		 	 
}
//检测24L01是否存在
//返回值:0，成功;1，失败	
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
  uint8_t buf_check[5];
	uint8_t i;	
  uint8_t data;
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
//  data = NRF_WRITE_REG+TX_ADDR;
//  HAL_SPI_Transmit(&hspi1,&data,1,1000);
//  HAL_SPI_Transmit(&hspi1,buf,5,1000);

  
	NRF24L01_Read_Buf(TX_ADDR,buf_check,5); //读出写入的地址  
//  data = TX_ADDR;
//  HAL_SPI_Receive(&hspi1,buf_check,5,1000);
	for(i=0;i<5;i++)if(buf_check[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}	 	 
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
   	NRF24L01_CSN_L;                 //使能SPI传输
  	status =SPI1_ReadWriteByte(reg);//发送寄存器号 
  	SPI1_ReadWriteByte(value);      //写入寄存器的值
  	NRF24L01_CSN_H;                 //禁止SPI传输	   
  	return(status);       			//返回状态值
}
//读取SPI寄存器值
//reg:要读的寄存器
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	NRF24L01_CSN_L;          //使能SPI传输		
  	SPI1_ReadWriteByte(reg);   //发送寄存器号
  	reg_val=SPI1_ReadWriteByte(0XFF);//读取寄存器内容
  	NRF24L01_CSN_H;          //禁止SPI传输		    
  	return(reg_val);           //返回状态值
}	
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
  uint8_t status,u8_ctr;	       
  NRF24L01_CSN_L;           //使能SPI传输
  status=SPI1_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值   	   
  for(u8_ctr=0;u8_ctr<len;u8_ctr++)
    pBuf[u8_ctr]=SPI1_ReadWriteByte(0XFF);//读出数据
  NRF24L01_CSN_H;       //关闭SPI传输
  return status;        //返回读到的状态值
}
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  uint8_t status,u8_ctr;	    
  NRF24L01_CSN_L;          //使能SPI传输
  status = SPI1_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
  for(u8_ctr=0; u8_ctr<len; u8_ctr++)
    SPI1_ReadWriteByte(*pBuf++); //写入数据	 
  NRF24L01_CSN_H;       //关闭SPI传输
  return status;          //返回读到的状态值
}				   
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
  
	
//  osDelay(100);
	while(NRF24L01_IRQ != 0 );//等待发送完成
	sta=NRF24L01_Read_Reg(NRF_READ_REG + STATUS);  //读取状态寄存器的值	   
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX)//达到最大重发次数
	{
//    HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
}
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;		    							      
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
		return 0; 
	}	   
	return 1;//没收到任何数据
}				

///
void NRF24L01_Mode(uint8_t mode)
{
	if(mode == IT_TX)
	{
		NRF24L01_CE_L;
		NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,IT_TX);
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0X7E); //清除所有中断,防止一进去发送模式就触发中断	
		NRF24L01_CE_H;
//		Delay_us(15);
	}
	else
	{
		NRF24L01_CE_L;
		NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,IT_RX);//配置为接收模式
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0X7E); //清除所有中断,防止一进去接收模式就触发中断
		NRF24L01_CE_H;

	}		
}

void NRF24L01_config(void)
{
  NRF24L01_CE_L;
  
  NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_AW, 0x03); //配置通信地址的长度，默认值时0x03,即地址长度为5字节
  NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH); //写TX节点地址 
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)TX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK
  NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1A); //设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次 0x1A

  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); //使能通道0自动应答
  NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度  
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //写RX节点地址
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40); //设置RF通道为40hz(1-64Hz都可以)
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); //设置TX发射参数,0db增益,2Mbps,低噪声增益关闭 （注意：低噪声增益关闭/开启直接影响通信,要开启都开启，要关闭都关闭0x0f）
  
  
  NRF24L01_Mode(IT_RX);
  NRF24L01_CE_H;
}


//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了		   
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE_L;	  
  	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  	 
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);	     //设置RF通信频率		  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  	NRF24L01_CE_H; //CE为高,进入接收模式 
}						 
//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE_L;	    
  	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //设置RF通道为40
    NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	NRF24L01_CE_H;//CE为高,10us后启动发送
}

 uint8_t Tx_buff[30] ;
uint8_t Rx_buff[30] ;
extern  uint16_t RC_ADC_Buff[4] ;


void nrf_sand_rc()
{

    static uint8_t DATA_ID = 0;
    DATA_ID ++;
    Tx_buff[0] = 0x01;
    Tx_buff[1] = 0xaa;
    
    Tx_buff[2] = RC_ADC_Buff[0]>>8;
    Tx_buff[3] = RC_ADC_Buff[0];
    
    Tx_buff[4] = RC_ADC_Buff[1]>>8;
    Tx_buff[5] = RC_ADC_Buff[1];
    
    Tx_buff[6] = RC_ADC_Buff[2]>>8;
    Tx_buff[7] = RC_ADC_Buff[2];
    
    Tx_buff[8] = RC_ADC_Buff[3]>>8;
    Tx_buff[9] = RC_ADC_Buff[3];
    
    Tx_buff[10] = DATA_ID;
  Tx_buff[11] = 0xa5;
 
}
extern uint8_t IRQ_timeout;
extern osSemaphoreId NRF_statusHandle;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t sta;
  IRQ_timeout = 0;
	sta=NRF24L01_Read_Reg(NRF_READ_REG + STATUS);  //读取状态寄存器的值	   
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX)//达到最大重发次数
	{
    NRF24L01_Mode(IT_RX);
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
	}
	if(sta&TX_OK)//发送完成
	{
    NRF24L01_Mode(IT_RX);
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
    osSemaphoreRelease(NRF_statusHandle);
	}
  
  if(sta & RX_OK)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
    NRF24L01_Read_Buf(RD_RX_PLOAD,Rx_buff,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
  }
}



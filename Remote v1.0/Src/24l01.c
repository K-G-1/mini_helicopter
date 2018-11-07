
#include "24l01.h"
#include "cmsis_os.h"

//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{		
//  uint8_t retry=0;				 	
//  retry = HAL_SPI_Transmit(&hspi1,&TxData,1,0x10);
//  HAL_SPI_Receive(&hspi1,&retry,1,0x10);
//  return retry; //����ͨ��SPIx������յ�����	
	
	static uint8_t txdata,rxdata;
	
	txdata=TxData;
	
	if(HAL_SPI_TransmitReceive(&hspi1,&txdata,&rxdata,1,0xff)!= HAL_OK)
	{
    return 0xff;	
	}
	
	return rxdata;	

}

    
const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0xAA,0xBB,0xCC,0x00,0x01}; //���͵�ַ
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0xAA,0xBB,0xCC,0x00,0x01};

//��ʼ��24L01��IO��
void NRF24L01_Init(void)
{ 	
//	GPIO_InitTypeDef GPIO_InitStructure;
//  SPI_InitTypeDef  SPI_InitStructure;

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PB,G�˿�ʱ��
//    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_SPI1,  ENABLE );//SPI2ʱ��ʹ�� 		
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				 //PB12���� ��ֹW25X�ĸ���
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);	//��ʼ��ָ��IO
// 	GPIO_SetBits(GPIOA,GPIO_Pin_4);//����				
// 	

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	//PG8 7 ���� 	  
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��ָ��IO
//  
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;   
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PB1 ����  
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1);//PG6,7,8����					 
//		 
//  SPI2_Init();    		//��ʼ��SPI	 
// 
//	SPI_Cmd(SPI1, DISABLE); // SPI���費ʹ��

//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI����Ϊ˫��˫��ȫ˫��
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//SPI����
//  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//���ͽ���8λ֡�ṹ
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//ʱ�����յ�
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//���ݲ����ڵ�1��ʱ����
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź����������
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//���ݴ����MSBλ��ʼ
//	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
//	SPI_Init(SPI1, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
// 
//	SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����
//		
  MX_SPI1_Init();
	NRF24L01_CE_L; 			//ʹ��24L01
	NRF24L01_CSN_H;			//SPIƬѡȡ��  
    
	 		 	 
}
//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
  uint8_t buf_check[5];
	uint8_t i;	
  uint8_t data;
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
//  data = NRF_WRITE_REG+TX_ADDR;
//  HAL_SPI_Transmit(&hspi1,&data,1,1000);
//  HAL_SPI_Transmit(&hspi1,buf,5,1000);

  
	NRF24L01_Read_Buf(TX_ADDR,buf_check,5); //����д��ĵ�ַ  
//  data = TX_ADDR;
//  HAL_SPI_Receive(&hspi1,buf_check,5,1000);
	for(i=0;i<5;i++)if(buf_check[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//���24L01����	
	return 0;		 //��⵽24L01
}	 	 
//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
   	NRF24L01_CSN_L;                 //ʹ��SPI����
  	status =SPI1_ReadWriteByte(reg);//���ͼĴ����� 
  	SPI1_ReadWriteByte(value);      //д��Ĵ�����ֵ
  	NRF24L01_CSN_H;                 //��ֹSPI����	   
  	return(status);       			//����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	NRF24L01_CSN_L;          //ʹ��SPI����		
  	SPI1_ReadWriteByte(reg);   //���ͼĴ�����
  	reg_val=SPI1_ReadWriteByte(0XFF);//��ȡ�Ĵ�������
  	NRF24L01_CSN_H;          //��ֹSPI����		    
  	return(reg_val);           //����״ֵ̬
}	
//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
  uint8_t status,u8_ctr;	       
  NRF24L01_CSN_L;           //ʹ��SPI����
  status=SPI1_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
  for(u8_ctr=0;u8_ctr<len;u8_ctr++)
    pBuf[u8_ctr]=SPI1_ReadWriteByte(0XFF);//��������
  NRF24L01_CSN_H;       //�ر�SPI����
  return status;        //���ض�����״ֵ̬
}
//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  uint8_t status,u8_ctr;	    
  NRF24L01_CSN_L;          //ʹ��SPI����
  status = SPI1_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  for(u8_ctr=0; u8_ctr<len; u8_ctr++)
    SPI1_ReadWriteByte(*pBuf++); //д������	 
  NRF24L01_CSN_H;       //�ر�SPI����
  return status;          //���ض�����״ֵ̬
}				   
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
  
	
//  osDelay(100);
	while(NRF24L01_IRQ != 0 );//�ȴ��������
	sta=NRF24L01_Read_Reg(NRF_READ_REG + STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
//    HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(sta&TX_OK)//�������
	{
		return TX_OK;
	}
	return 0xff;//����ԭ����ʧ��
}
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;		    							      
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&RX_OK)//���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
		return 0; 
	}	   
	return 1;//û�յ��κ�����
}				

///
void NRF24L01_Mode(uint8_t mode)
{
	if(mode == IT_TX)
	{
		NRF24L01_CE_L;
		NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,IT_TX);
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0X7E); //��������ж�,��ֹһ��ȥ����ģʽ�ʹ����ж�	
		NRF24L01_CE_H;
//		Delay_us(15);
	}
	else
	{
		NRF24L01_CE_L;
		NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,IT_RX);//����Ϊ����ģʽ
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0X7E); //��������ж�,��ֹһ��ȥ����ģʽ�ʹ����ж�
		NRF24L01_CE_H;

	}		
}

void NRF24L01_config(void)
{
  NRF24L01_CE_L;
  
  NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_AW, 0x03); //����ͨ�ŵ�ַ�ĳ��ȣ�Ĭ��ֵʱ0x03,����ַ����Ϊ5�ֽ�
  NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH); //дTX�ڵ��ַ 
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)TX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK
  NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1A); //�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10�� 0x1A

  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); //ʹ��ͨ��0�Զ�Ӧ��
  NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��  
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //дRX�ڵ��ַ
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40); //����RFͨ��Ϊ40hz(1-64Hz������)
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); //����TX�������,0db����,2Mbps,����������ر� ��ע�⣺����������ر�/����ֱ��Ӱ��ͨ��,Ҫ������������Ҫ�رն��ر�0x0f��
  
  
  NRF24L01_Mode(IT_RX);
  NRF24L01_CE_H;
}


//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE_L;	  
  	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
	  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  	 
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);	     //����RFͨ��Ƶ��		  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  	NRF24L01_CE_H; //CEΪ��,�������ģʽ 
}						 
//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	 
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE_L;	    
  	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //����RFͨ��Ϊ40
    NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	NRF24L01_CE_H;//CEΪ��,10us����������
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
	sta=NRF24L01_Read_Reg(NRF_READ_REG + STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
    NRF24L01_Mode(IT_RX);
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
	}
	if(sta&TX_OK)//�������
	{
    NRF24L01_Mode(IT_RX);
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
    osSemaphoreRelease(NRF_statusHandle);
	}
  
  if(sta & RX_OK)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
    NRF24L01_Read_Buf(RD_RX_PLOAD,Rx_buff,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
  }
}



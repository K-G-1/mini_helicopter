#include "usart.h"
#include <stdarg.h>
#include "DataTransferV26.h"
char Rec_Bluetooth_Buf[8];
uint8_t j = 0;


#define USART1_PORT_CLK	RCC_APB2Periph_GPIOA
#define USART1_PORT		GPIOA
#define USART1_TX		GPIO_Pin_9
#define USART1_RX		GPIO_Pin_10

#define USART2_PORT_CLK	RCC_APB2Periph_GPIOA
#define USART2_PORT		GPIOA
#define USART2_TX		GPIO_Pin_2
#define USART2_RX		GPIO_Pin_3


u8 Rx_Buf[32];
u8 Rx_Adr=0;
/*
 * 函数名：USART1_Config
 * 描述  ：USART1 GPIO 配置,工作模式配置。115200 8-N-1
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	/* config USART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | USART1_PORT_CLK, ENABLE);
	
	/* USART1 GPIO config */
	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = USART1_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART1_PORT, &GPIO_InitStructure);    
	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = USART1_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USART1_PORT, &GPIO_InitStructure);
	  
	/* USART1 mode config */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	  	/* 使能串口1接收中断 */
			
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //???????
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK??????????->???
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //?????????????
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //?????????????SCLK??
	
	USART_Init(USART1, &USART_InitStructure);
	USART_ClockInit(USART1, &USART_ClockInitStruct);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);


	USART_Cmd(USART1, ENABLE);

//	USART_ClearFlag( USART1, USART_FLAG_TC );
//	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}

/*
 * 函数名：USART2_Config
 * 描述  ：USART2 GPIO 配置,工作模式配置。115200 8-N-1
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void USART2_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* config USART2 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(USART2_PORT_CLK, ENABLE);
	
	/* USART2 GPIO config */
	/* Configure USART2 Tx (PA2) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = USART2_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART2_PORT, &GPIO_InitStructure);    
	/* Configure USART2 Rx (PA3) as input floating */
	GPIO_InitStructure.GPIO_Pin = USART2_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USART2_PORT, &GPIO_InitStructure);
	  
	/* USART2 mode config */
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);  

  	/* 使能串口2接收中断 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART2, ENABLE);
	USART_ClearFlag( USART2, USART_FLAG_TC );
	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}

/*
 * 函数名：fputc
 * 描述  ：重定向c库函数printf到USART
 * 输入  ：无
 * 输出  ：无
 * 调用  ：由printf调用
 */
int fputc(int ch, FILE *f)
{
	/* 将Printf内容发往串口 */
	USART_SendData(USART2, (unsigned char) ch);
//	while (!(USART2->SR & USART_FLAG_TXE));
	while( USART_GetFlagStatus(USART2,USART_FLAG_TC)!= SET);	
	return (ch);
}

/*
 * 函数名：itoa
 * 描述  ：将整形数据转换成字符串
 * 输入  ：-radix =10 表示10进制，其他结果为0
 *         -value 要转换的整形数
 *         -buf 转换后的字符串
 *         -radix = 10
 * 输出  ：无
 * 返回  ：无
 * 调用  ：被USART_printf()调用
 */
static char *itoa(int value, char *string, int radix)
{
	int     i, d;
	int     flag = 0;
	char    *ptr = string;
	
	/* This implementation only works for decimal numbers. */
	if (radix != 10)
	{
	    *ptr = 0;
	    return string;
	}
	
	if (!value)
	{
	    *ptr++ = 0x30;
	    *ptr = 0;
	    return string;
	}
	
	/* if this is a negative value insert the minus sign. */
	if (value < 0)
	{
	    *ptr++ = '-';
	
	    /* Make the value positive. */
	    value *= -1;
	}
	
	for (i = 10000; i > 0; i /= 10)
	{
	    d = value / i;
	
	    if (d || flag)
	    {
	        *ptr++ = (char)(d + 0x30);
	        value -= (d * i);
	        flag = 1;
	    }
	}
	
	/* Null terminate the string. */
	*ptr = 0;
	
	return string;

} /* NCL_Itoa */

/*
 * 函数名：USART_printf
 * 描述  ：格式化输出，类似于C库中的printf，但这里没有用到C库
 * 输入  ：-USARTx 串口通道，这里只用到了串口2，即USART2
 *		     -Data   要发送到串口的内容的指针
 *			   -...    其他参数
 * 输出  ：无
 * 返回  ：无 
 * 调用  ：外部调用
 *         典型应( USART2, "\r\n this is a demo \r\n" );
 *            		 USART_printf( USART2, "\r\n %d \r\n", i );
 *            		 USART_printf( USART2, "\r\n %s \r\n", j );
 */

void USART_printf(USART_TypeDef* USARTx, uint8_t *Data,...)
{
	const char *s;
	int d;   
	char buf[16];
	
	va_list ap;
	va_start(ap, Data);
	
	while ( *Data != 0)     // 判断是否到达字符串结束符
	{				                          
		if ( *Data == 0x5c )  //'\'
	{									  
	switch ( *++Data )
	{
		case 'r':							          //回车符
			USART_SendData(USARTx, 0x0d);
			Data ++;
		break;
		
		case 'n':							          //换行符
			USART_SendData(USARTx, 0x0a);	
			Data ++;
		break;
		
		default:
			Data ++;
		break;
	}			 
	}
	else if ( *Data == '%')
	{									  //
	switch ( *++Data )
	{				
		case 's':										  //字符串
			s = va_arg(ap, const char *);
	for ( ; *s; s++) 
	{
		USART_SendData(USARTx,*s);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
		Data++;
		break;
	
	case 'd':										//十进制
	d = va_arg(ap, int);
	itoa(d, buf, 10);
	for (s = buf; *s; s++) 
	{
		USART_SendData(USARTx,*s);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
	Data++;
	break;
		 default:
				Data++;
		    break;
	}		 
	} /* end of else if */
	else USART_SendData(USARTx, *Data++);
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
} 

void clearBluetooth_Buff(void)
{
	USART_ITConfig(USART_DEBUG, USART_IT_RXNE, DISABLE);	 //取消USART3的接收中断

    for(j=0;j<Bluetooth_Max;j++)    //将缓存内容清零
    {
		Rec_Bluetooth_Buf[j] = 0;
	}
    j = 0;                    //接收字符串的起始存储位置
	USART_ITConfig(USART_DEBUG, USART_IT_RXNE, ENABLE); //打开USART3的接收中断
}
u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0;
//static u8 RxBuffer[50];
//static u8 RxState = 0;

void g_v_Uart1_IRQ(void)
{	
	

	if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)//??!????if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)???
    {
        u8 com_data = USART1->DR;
			USART_ClearFlag(USART3,USART_IT_ORE);
    }
		
	//发送中断
	if((USART1->SR & (1<<7))&&(USART1->CR1 & USART_CR1_TXEIE))//if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET)
	{
		USART1->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{    
			USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE中断
			//USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
		}
	}
	
	//接收中断
if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	 {
		u8 com_data = USART1->DR;
		ANO_DT_Data_Receive_Prepare(com_data);
	 }
}

void g_v_Uart1PutBuf(unsigned char *DataToSend , uint8_t data_num)
{
	uint8_t i=0;
	for(i=0;i<data_num;i++)
	{
		TxBuffer[count++] = *(DataToSend+i);
	}
	if(!(USART1->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
}



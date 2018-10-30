#include "delay.h"

/**************************************
延时X微秒(STC12C5A60S2@12M)
不同的工作环境,需要调整此函数
此延时函数是使用1T的指令周期进行计算,与传统的12T的MCU不同
**************************************/
void DelayXus(uint8_t n)
{
    while (n--)
    {
//        _nop_();
//        _nop_();
    }
}

void delayms(uint16_t xms)
{
	uint16_t i,j;
	for(i=xms;i>0;i--)
		for(j=110;j>0;j--);
}

void delay2(uint16_t ms)// 延时子程序
{
    uint8_t i;
    while(ms--)
    {
        for(i=0;i<120;i++);
    }
} 

void  Delay_10us(void)
{      
	unsigned char i;
	i = 27;
	while (--i);
}

///*----------------------------
//Software delay function
//----------------------------*/
//void Delay(uint16_t n)
//{
//    uint16_t x;

//    while (n--)
//    {
//        x = 5000;
//        while (x--);
//    }
//}

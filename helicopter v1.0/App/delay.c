#include "delay.h"

/**************************************
��ʱX΢��(STC12C5A60S2@12M)
��ͬ�Ĺ�������,��Ҫ�����˺���
����ʱ������ʹ��1T��ָ�����ڽ��м���,�봫ͳ��12T��MCU��ͬ
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

void delay2(uint16_t ms)// ��ʱ�ӳ���
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

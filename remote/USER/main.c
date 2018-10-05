#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "24l01.h"

u8 Tx_buff[30] ={0};

int main(void)
{	 
    u8 sta;
    u8 cnt;
    delay_init();	    	 //��ʱ������ʼ��	  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
    uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
    LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
    NRF24L01_Init();
    delay_ms(1000);
    while(NRF24L01_Check() != 0)
    {
        LED1 = 0;
        delay_ms(1000);
    }
    LED1 = 1;
    NRF24L01_TX_Mode();

    Tx_buff[0] = 0xaa;
    Tx_buff[1] = 0xaa;
    Tx_buff[2] = 0x02;

    while(1)
    {
        cnt++;
        if(cnt >=200)
        {
            cnt = 0;
        }
//        LED0 = 0;
//        delay_ms(1000);
//        LED0 = 1;
//        delay_ms(1000);
        Tx_buff[4] = cnt;
        if(NRF24L01_TxPacket(Tx_buff)==TX_OK)
        {
            LED1 = 0;
        }
        else
            LED1 = 1;
        delay_ms(1500);				   
//        sta=NRF24L01_Read_Reg(NRF_READ_REG + STATUS);
//        printf("sta = %d",sta);
        LED0 = !LED0;
    }
}

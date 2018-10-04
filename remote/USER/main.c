#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "24l01.h"

u8 Tx_buff[30] = "2401 tx";

int main(void)
{	 
    u8 sta;
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

    while(1)
    {
//        LED0 = 0;
//        delay_ms(1000);
//        LED0 = 1;
//        delay_ms(1000);
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

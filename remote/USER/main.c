#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "24l01.h"
#include "ADC.h"
#include "timer3.h"
#include "oled.h"


u8 Tx_buff[30] ={0};
uint16_t Offest_data[4] ={100,100,100,100};
extern uint16_t RC_ADC_Buff[4] ;


int main(void)
{	 
    u8 sta;
    uint16_t cnt;
    delay_init();	    	 //��ʱ������ʼ��	  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
    uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
    LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
    Adc_Init();
    
    
//    NRF24L01_Init();
//    delay_ms(1000);
//    
//    while(NRF24L01_Check() != 0)
//    {
//        LED1 = 0;
//        delay_ms(1000);
//    }
//    LED1 = 1;
//    NRF24L01_TX_Mode();

    OLED_Init();
    oled_dis_str();
    oled_show_offest_data(Offest_data);
    tim3_init(1000-1,720-1); //100HZ

    while(1)
    {
        oled_show_RC_data(RC_ADC_Buff);
        
        delay_ms(100);

        LED0 = !LED0;

    }
}





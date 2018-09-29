#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"




int main(void)
{	 

    delay_init();	    	 //��ʱ������ʼ��	  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
    uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
    LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
  

    while(1)
    {
        LED0 = 0;
        LED1 = 0;
        
        delay_ms(1000);
        LED0 = 1;
        LED1 = 1;
        delay_ms(1000);

    }
}

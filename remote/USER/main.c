#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"




int main(void)
{	 

    delay_init();	    	 //延时函数初始化	  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
    uart_init(115200);	 	//串口初始化为115200
    LED_Init();		  		//初始化与LED连接的硬件接口
  

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

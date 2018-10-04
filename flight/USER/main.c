#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "IIC.h"
#include "mpu9250.h"
#include "mpu6050.h"
#include "sand_data.h"
#include "timer3.h"
#include  "IMU.h"
#include "pwm.h"
#include "spi.h"
#include "24l01.h"

u8 Tx_buff[33] = "2401 tx";
u8 Rx_buff[33];
int main(void)
{	 
    u8 sta;
    int t=0;
    delay_init();	    	 //延时函数初始化	  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
    uart_init(115200);	 	//串口初始化为115200
    LED_Init();		  		//初始化与LED连接的硬件接口
  
    IIC_Init();
    mpu9250_init();
    Get_offest();
    pwm_init(2500-1,72-1);
    tim3_init(100-1,7200-1);
    NRF24L01_Init();
    delay_ms(1000);
    while(NRF24L01_Check() != 0)
    {
        LED2 = 0;
        delay_ms(1000);
    }
    LED2 = 1;
    NRF24L01_RX_Mode();
    while(1)
    {
//        SPI2_SetSpeed(SPI_BaudRatePrescaler_8);
//        sta=NRF24L01_Read_Reg(STATUS);
//        NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
//        if(NRF24L01_RxPacket(Rx_buff)==0)//一旦接收到信息,则显示出来.
//        {

//            LED1 = 0;
//            t = 0;
//        }  
//        else
//        {         
//            t++;
//            if(t >=1000)
//                LED1 = 1;
//        }
            
    }
}

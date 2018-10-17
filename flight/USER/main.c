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
int IRQ_timeout = 0;
int main(void)
{	 
    u8 sta;
                                                                                                                                                            int t=0;
    delay_init();	    	 //延时函数初始化	  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
    uart_init(115200);	 	//串口初始化为115200
    LED_Init();		  		//初始化与LED连接的硬件接口
  
    IIC_Init();
    while(mpu6050_init())
        LED1 = !LED1;
    Get_6050_offest();
    pwm_init(1200-1,72-1); //1Khz
    tim4_init(2000-1,72-1);

//    NRF24L01_Init();
//    while(NRF24L01_Check() != 0)
//    {
//        LED2 = 0;
//        delay_ms(1000);
//    }
//    LED2 = 1;
//    NRF24L01_RX_Mode();
//    NRF_IRQ_INIT();
    TIM_Cmd(TIM4, ENABLE);

    Moto_PwmRflash(500,500,200,200);
    while(1)
    {
        
//        READ_MPU6050_ACCEL();
//        READ_MPU6050_GYRO();
        
        
//        IRQ_timeout ++;
//        delay_ms(100);
//        if(IRQ_timeout >= 2000)
//        {
//            IRQ_timeout = 0;
//            sta=NRF24L01_Read_Reg(STATUS);
//            NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
//            if(sta&RX_OK)//接收到数据
//            {
//                NRF24L01_Read_Buf(RD_RX_PLOAD,Rx_buff,RX_PLOAD_WIDTH);//读取数据
//                NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 

//            }
//            
//        }
//        SPI2_SetSpeed(SPI_BaudRatePrescaler_8);
        
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

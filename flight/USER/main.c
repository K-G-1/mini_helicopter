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
    delay_init();	    	 //��ʱ������ʼ��	  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
    uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
    LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
  
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
//            NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
//            if(sta&RX_OK)//���յ�����
//            {
//                NRF24L01_Read_Buf(RD_RX_PLOAD,Rx_buff,RX_PLOAD_WIDTH);//��ȡ����
//                NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 

//            }
//            
//        }
//        SPI2_SetSpeed(SPI_BaudRatePrescaler_8);
        
//        NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
//        if(NRF24L01_RxPacket(Rx_buff)==0)//һ�����յ���Ϣ,����ʾ����.
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

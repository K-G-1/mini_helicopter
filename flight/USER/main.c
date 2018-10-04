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
    delay_init();	    	 //��ʱ������ʼ��	  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
    uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
    LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
  
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

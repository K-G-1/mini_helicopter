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


int main(void)
{	 

    delay_init();	    	 //��ʱ������ʼ��	  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
    uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
    LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
  
    IIC_Init();
    mpu9250_init();
    Get_offest();
    pwm_init(2500-1,72-1);
    tim3_init(100-1,7200-1);
//    Moto_PwmRflash(100,100,100,100);
    while(1)
    {
        if(angle.pitch >40||angle.pitch<-40)
            PWM3  = 1;
        else 
            PWM3 = 0;
//        LED0 = 0;
//        LED1 = 0;
//        
//        delay_ms(1000);
//        LED0 = 1;
//        LED1 = 1;
//        delay_ms(1000);
//        READ_9250();
////        sand_ACC_GYRO_data(); 
//        READ_MPU9250_MAG();
    }
}

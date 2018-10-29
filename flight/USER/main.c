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
#include "rc.h"
#include "control.h"
#include "stmflash.h"
#include "param.h"
u8 Tx_buff[33] = "2401 tx";
u8 Rx_buff[33];
extern int IRQ_timeout;
#define FLASH_SAVE_ADDR  0X0800F000		//����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000)

void param_init()
{
    Rc_Data.THROTTLE = 1000;
    Rc_Data.PITCH = 1500;
    Rc_Data.ROLL = 1500;
    Rc_Data.YAW = 1500;
    
    Rc_Data.pitch_offset = 1500;
    Rc_Data.roll_offset = 1500;
    Rc_Data.yaw_offset = 1500;

    Read_Acc_Gyro_offest();
    Read_PID_shell();
    Read_PID_core();
    mode = 1;
 
}
int main(void)
{	 
   
    u8 sta;
                                                                                                                                                            
    int t=0;
//    NVIC_SetVectorTable(0x8002800,0);
    delay_init();	    	 //��ʱ������ʼ��	  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
    Jtag_disable();
    uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
    LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�

    Jtag_disable();
    NRF24L01_Init();
    while(NRF24L01_Check() != 0)
    {
        LED2 = 0;
        delay_ms(1000);
    }
    LED2 = 1;
    NRF24L01_RX_Mode();
	Jtag_disable();
    NRF_IRQ_INIT();
    
    
    Jtag_disable();
    IIC_Init();
#if USE_IMU_DEVICE
    while(mpu6050_init())
        LED1 = !LED1;
//    Get_6050_offest();
	tim4_init(2000-1,72-1);
	tim2_init(1000-1,72-1);
#else	
	while(mpu9250_init())
	LED1 = !LED1;
	Get_offest();
	tim2_init(1000-1,72-1);
	tim4_init(1000-1,7200-1);
#endif	
	Jtag_disable();
	pwm_init(4000-1,1-1); //1Khz
    Moto_PwmRflash(0,0,0,0);
    
    param_init();

#if USE_IMU_DEVICE	
	TIM_Cmd(TIM4, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
#else
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
#endif
    while(1)
    {
        
    }
}

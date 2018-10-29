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
#define FLASH_SAVE_ADDR  0X0800F000		//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)

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
    delay_init();	    	 //延时函数初始化	  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
    Jtag_disable();
    uart_init(115200);	 	//串口初始化为115200
    LED_Init();		  		//初始化与LED连接的硬件接口

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
